package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.WristTrajFollower;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ArmvatorPosition;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class Superstructure {
  public static enum RunMode {
    PROFILED_PID, VOLTAGE, PID
  }

  Pivot pivot;
  Telescope telescope;
  Wrist wrist;

  ArmvatorSample lastSample;
  public Superstructure(Pivot pivot, Telescope telescope, Wrist wrist) {
    this.pivot = pivot;
    this.telescope = telescope;
    this.wrist = wrist;
    telescope.setPivotAngleRadSupplier(pivot::getAngleRad);
  }

  private void followSample(ArmvatorSample sample) {
    pivot.setMode(RunMode.PID);
    telescope.setMode(RunMode.PID);
    pivot.setSample(sample); 
    telescope.setSample(sample);
    lastSample = sample;
  }
  
  private boolean atPidSetpoint() {
    return pivot.atPidSetpoint() && telescope.atPidSetpoint();
  }

  public Command followAvTrajectory(ArmvatorTrajectory traj) {
    return traj.follow(this::followSample, this::atPidSetpoint, true, true, pivot, telescope);
  }

  public Command goTo(SuperstructurePosition pos) {
    var currentPos = ArmvatorPosition.getNearest(getEndEffPos());
    var traj = ArmvatorTrajectory.load(currentPos, pos.armvatorPosition);
    return followAvTrajectory(traj).deadlineFor(new WristTrajFollower(traj, pos, wrist, () -> lastSample));
  }

  @AutoLogOutput
  public Translation2d getEndEffPos() {
    double x = telescope.getPosition() * Math.cos(pivot.getAngle().getRadians());
    double y = telescope.getPosition() * Math.sin(pivot.getAngle().getRadians());
    return new Translation2d(x, y);
  }

  public ArmvatorSample getLastSample() {
    return lastSample;
  }

}
