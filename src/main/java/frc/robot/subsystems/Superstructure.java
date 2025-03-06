package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ArmvatorPosition;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class Superstructure {
  public static enum RunMode {
    PROFILED_PID, VOLTAGE, TRAJECTORY
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
    pivot.setMode(RunMode.TRAJECTORY);
    telescope.setMode(RunMode.TRAJECTORY);
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
    Logger.recordOutput("Superstructure/goToFrom", currentPos);
    Logger.recordOutput("Superstructure/goToTo", pos);
    return followAvTrajectory(traj);
  }

  @AutoLogOutput
  public Translation2d getEndEffPos() {
    double x = telescope.getTotalLength() * Math.cos(pivot.getAngle().getRadians());
    double y = telescope.getTotalLength() * Math.sin(pivot.getAngle().getRadians());
    return new Translation2d(x, y);
  }

  public ArmvatorSample getLastSample() {
    return lastSample;
  }

  @AutoLogOutput
  public ArmvatorPosition getClosestArmvatorPosition() {
    return ArmvatorPosition.getNearest(getEndEffPos());
  }

}
