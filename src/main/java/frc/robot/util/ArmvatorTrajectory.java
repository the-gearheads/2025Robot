package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public record ArmvatorTrajectory(String name, List<ArmvatorSample> samples) {
  
  private static final Gson GSON = new Gson();

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
  public Command follow(Consumer<ArmvatorSample> consumer, Subsystem... requirements) {
    Timer timer = new Timer();
    return new FunctionalCommand(
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
  }

  public static ArmvatorTrajectory load(String filename) {
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
        sample.get("elevator_accel").getAsDouble(),
        new Translation2d(
          sample.getAsJsonArray("endeff_pos").get(0).getAsDouble(),
          sample.getAsJsonArray("endeff_pos").get(1).getAsDouble()
        )
      ));
    }

    return new ArmvatorTrajectory(name, samples);
  }

  public static ArmvatorTrajectory load(ArmvatorPosition start, ArmvatorPosition end) {
    return load(start.toString() + "," + end.toString());
  }

  
}
