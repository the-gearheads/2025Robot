package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructurePosition;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class WristTrajFollower extends Command {
  Wrist wrist;
  ArmvatorTrajectory traj;
  SuperstructurePosition endPos;
  Supplier<ArmvatorSample> lastSampleSupplier;

  public WristTrajFollower(ArmvatorTrajectory traj, SuperstructurePosition endPos, Wrist wrist, Supplier<ArmvatorSample> lastSampleSupplier) {
    this.wrist = wrist;
    this.traj = traj;
    this.endPos = endPos;
    this.lastSampleSupplier = lastSampleSupplier;
    addRequirements(wrist);
  }

  @Override
  public void execute() {
    if (lastSampleSupplier.get().t() > (traj.getDuration() / 3.0)) {
      wrist.setGoal(endPos.wristAngle);
    }
  }
}
