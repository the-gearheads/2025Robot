// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.MiscConstants;
import frc.robot.util.TriConsumer;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private DigitalInput brakeCoastButton = new DigitalInput(MiscConstants.BREAK_COAST_BUTTON_PORT);
  private Debouncer brakeCoastButtonDebouncer = new Debouncer(0.05);
  private boolean lastBrakeCoastButton = false;
  private boolean isBraken = true;


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @SuppressWarnings("resource")
  public Robot() {
    boolean isReplay = false;

    if (!isReplay || isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.registerURCL(URCL.startExternal());
    LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
    Logger.start();


    /* Log all commands running, both uniquely and by name. */
    Map<String, Integer> commandCounts = new HashMap<>();
    TriConsumer<Command, Boolean, String> logCommandFunction = (Command command, Boolean active, String reason) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      commandCounts.put(name, count);
      final boolean[] isDefault = {false}; // I hate copilot for suggesting this as a workaround but it does in fact work
      String reqs = command.getRequirements()
        .stream()
        .map(subsystem -> {
          if(subsystem.getDefaultCommand() == command) {
            isDefault[0] = true;
          }
          return subsystem.getName();
        })
        .collect(Collectors.joining("_"));
      Logger.recordOutput("RunningCommands/Unique/" + name + (isDefault[0] ? "_DEFAULT_" : "_") + reqs + "_" + Integer.toHexString(command.hashCode()), reason);
      Logger.recordOutput("RunningCommands/All/" + name, count > 0);
    };

    CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
      logCommandFunction.accept(command, true, "RUNNING");
    });

    CommandScheduler.getInstance().onCommandFinish((Command command) -> {
      logCommandFunction.accept(command, false, "FINISHED");
    });

    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
        logCommandFunction.accept(command, false, "INTERRUPTED");
    });


    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    CanandEventLoop.getInstance();
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    robotContainer.configureBindings();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    boolean output = brakeCoastButtonDebouncer.calculate(!brakeCoastButton.get());
    // m_robotContainer.leds.setState(LedState.RAINBOW);
    if(output && !lastBrakeCoastButton) {
      isBraken = !isBraken;
      robotContainer.setAllBrakeCoast(isBraken);
      System.out.println("Brake/Coast: " + (isBraken ? "Brake" : "Coast"));
    }

    lastBrakeCoastButton = output;

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    if(!isBraken) {
      isBraken = true;
      lastBrakeCoastButton = true;
      robotContainer.setAllBrakeCoast(true);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    if(!isBraken) {
      isBraken = true;
      lastBrakeCoastButton = true;
      robotContainer.setAllBrakeCoast(true);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1); // default is red grrr
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(robotContainer.getCurrentDrawSim())
    );
  }
}
