package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public record ArmvatorTrajectory(String name, List<ArmvatorSample> samples) {
  
  private static final Gson GSON = new Gson();
  private static final Map<String, ArmvatorTrajectory> trajectoryCache = new HashMap<>();

  public ArmvatorSample getInitialSample() {
    if(samples.isEmpty()) {
      return null;
    }
    return samples.get(0);
  }

  public ArmvatorSample getFinalSample() {
    if(samples.isEmpty()) {
      return null;
    }
    return samples.get(samples.size() - 1);
  }

  public double getDuration() {
    if(samples.isEmpty()) {
      return 0;
    }

    /* The last sample should run for dt time also. Since we can probably safely assume the target waypoint of the size-2 is the same as the one for size-1 the dt should be correct. */
    double dt_additional = 0;
    if(samples.size() > 1) {
      dt_additional = samples.get(samples.size() - 1).t() - samples.get(samples.size() - 2).t();
    }

    return samples.get(samples.size() - 1).t() + dt_additional;
  }

  // basically just https://github.com/SleipnirGroup/Choreo/blob/1f68147d11f0f59e00a038bf7dda2b4f3152da7d/choreolib/src/main/java/choreo/trajectory/Trajectory.java#L105
  public ArmvatorSample sampleAt(double t) {
    if(samples.isEmpty()) {
      return null;
    }

    if(t <= 0) {
      return getInitialSample();
    }

    if(t >= getDuration()) {
      return getFinalSample();
    }

    // binary search for samples both before and after t
    int low = 0;
    int high = samples.size() - 1;
    while(low < high) {
      int mid = (low + high) / 2;
      if(samples.get(mid).t() < t) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }

    if(low == 0) {
      return samples.get(0);
    }

    ArmvatorSample before = samples.get(low - 1);
    ArmvatorSample after = samples.get(low);
    return before.interpolate(after, t);
  }
  
  // Follows an armvator trajectory, sampling periodically. Consumer is expected to actually do the following.
  public Command follow(Consumer<ArmvatorSample> consumer, BooleanSupplier atSetpoint, boolean waitUntilAtStart, boolean waitUntilAtEnd, Subsystem... requirements) {
    Timer timer = new Timer();

    var gotoStartComamnd = Commands.run(() -> {consumer.accept(sampleAt(0));}, requirements).until(atSetpoint).withName("ArmTrajFollowerGotoStart " + name).withTimeout(2);
    var waitUntilEndCommand = Commands.run(() -> {consumer.accept(sampleAt(getDuration()));}, requirements).until(atSetpoint).withName("ArmTrajFollowerWaitUntilEnd " + name).withTimeout(2);
    var trajCommand = new FunctionalCommand(
      // On init
      () -> {
        timer.start();
      },
      // On periodic
      () -> {
        double t = timer.get();
        ArmvatorSample sample = sampleAt(t);
        consumer.accept(sample);
      },
      // On end
      end -> {
        // I think it might be possible to skip the final sample in the case of loop overruns? That'd be bad.
        if(end) {
          consumer.accept(getFinalSample());
        }
        timer.stop();
        timer.reset();
      },
      // Is finished
      () -> {
        return timer.get() > getDuration();
      },
      requirements
    );

    var sequence = new SequentialCommandGroup();
    if(waitUntilAtStart) {
      sequence.addCommands(gotoStartComamnd);
    }
    sequence.addCommands(trajCommand);
    if(waitUntilAtEnd) {
      sequence.addCommands(waitUntilEndCommand);
    }
    return sequence.withName("ArmvatorTrajFollower " + name);
  }


  /**
   * Loads ArmvatorTrajectory from file. Slow.
   * @param filename trajectory name
   * @return armvator trajectory
   */
  private static ArmvatorTrajectory load(String filename) {
    File deployDir = Filesystem.getDeployDirectory();
    File file = new File(deployDir, "avtrajopt/" + filename + ".agentraj"); // json
    if (!file.exists()) {
      DriverStation.reportError("File not found: " + file.getAbsolutePath(), true);
    }

    Reader reader = null;
    try {
      reader = new FileReader(file);
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    JsonObject json = GSON.fromJson(reader, JsonObject.class);
    String name = json.get("name").getAsString();
    var samplesJson = json.getAsJsonArray("samples");
    List<ArmvatorSample> samples = new ArrayList<>(samplesJson.size());
    for (var element : samplesJson) {
      var sample = element.getAsJsonObject();
      samples.add(new ArmvatorSample(
        sample.get("time").getAsDouble(),
        sample.get("sample_num").getAsDouble(),
        sample.get("pivot_angle").getAsDouble(),
        sample.get("pivot_velocity").getAsDouble(),
        sample.get("elevator_length").getAsDouble(),
        sample.get("elevator_velocity").getAsDouble(),
        sample.get("pivot_accel").getAsDouble(),
        sample.get("elevator_accel").getAsDouble()
      ));
    }

    return new ArmvatorTrajectory(name, samples);
  }

  /**
   * Loads all trajectories from file into the trajectory cache. Needs to be called before using any trajectories.
   */
  public static void loadAll() {
    if(!trajectoryCache.isEmpty()) {
      return;
    }
    File deployDir = Filesystem.getDeployDirectory();
    File trajDir = new File(deployDir, "avtrajopt");

    for (File file : trajDir.listFiles()) {
      if (file.getName().endsWith(".agentraj")) {
        String name = file.getName().replace(".agentraj", "");
        trajectoryCache.put(name, load(name));
      }
    }
  }

  /**
   * Loads from cache instead of from file
   * @param start start position
   * @param end endposition
   * @return
   */
  public static ArmvatorTrajectory load(ArmvatorPosition start, ArmvatorPosition end) {
    if(trajectoryCache.isEmpty()) {
      DriverStation.reportError("Trajectory cache is empty, call loadAll() plz, thx", true);
      // eh whats the point of throwing an error properly if things are gonna crash anyways
    }

    String name = start.name() + "," + end.name();
    if (!trajectoryCache.containsKey(name)) {
      DriverStation.reportError("Trajectory not found: " + name, true);
    }

    return trajectoryCache.get(name);
  }

  
}
