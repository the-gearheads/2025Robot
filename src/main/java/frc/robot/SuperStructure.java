package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class SuperStructure {
  public static enum RunMode {
    PROFILED_PID, VOLTAGE, PID
  }

  Pivot pivot;
  Telescope telescope;
  Wrist wrist;
  public SuperStructure(Pivot pivot, Telescope telescope) {
    this.pivot = pivot;
    this.telescope = telescope;
  }

  private void followSample(ArmvatorSample sample) {
    pivot.setMode(RunMode.PID);
    telescope.setMode(RunMode.PID);
    pivot.setSample(sample); 
    telescope.setSample(sample);
  }

  public Command followTrajectory(ArmvatorTrajectory traj) {
    return traj.follow(this::followSample, pivot, telescope);
  }

  public Translation2d getEndEffPos() {
    double x = telescope.getPosition() * Math.cos(pivot.getAngle().getRadians());
    double y = telescope.getPosition() * Math.sin(pivot.getAngle().getRadians());
    return new Translation2d(x, y);
  }

}
