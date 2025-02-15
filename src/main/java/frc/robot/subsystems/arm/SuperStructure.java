package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.ELEVATOR_FEEDFORWARD;
import static frc.robot.constants.ArmConstants.PIVOT_FEEDFORWARD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.ManualTelescope;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class SuperStructure {
  public final Pivot pivot;
  public final Telescope telescope;

  private double ffPivot;
  private double outputPivot;
  private double ffTelescope;
  private double outputTelescope;

  public static enum RunMode {
    PID, VOLTAGE, TRAJECTORY
  }

  public SuperStructure() {
    if (Robot.isReal()) {
      pivot = new Pivot();
      telescope = new Telescope();
    } else {
      pivot = new PivotSim();
      telescope = new telescopeSim();
    }

    pivot.setDefaultCommand(new ManualPivot(pivot));
    telescope.setDefaultCommand(new ManualTelescope(telescope));
  }

  public void followSample(ArmvatorSample sample) {
    pivot.setSample(sample); 
  }

  public Command followTrajectory(ArmvatorTrajectory traj) {
    return traj.follow(this::followSample, pivot, telescope);
  }
}
