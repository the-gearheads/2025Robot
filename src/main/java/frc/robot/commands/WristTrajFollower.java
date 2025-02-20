package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristPositions;
import frc.robot.util.ArmvatorPosition;
import frc.robot.util.ArmvatorTrajectory;

public class WristTrajFollower extends Command {
  Wrist wrist;
  SuperStructure superStructure;
  ArmvatorTrajectory traj;
  ArmvatorPosition endPos;

  public WristTrajFollower(ArmvatorTrajectory traj, Wrist wrist, SuperStructure superStructure) {
    this.wrist = wrist;
    this.superStructure = superStructure;
    this.traj = traj;
    endPos = ArmvatorPosition.getNearest(traj.getFinalSample().endeffPos());
    addRequirements(wrist);
  }

  @Override
  public void execute() {
    if (superStructure.getLastSample().t() > (traj.getDuration() / 2.0)) {
      wrist.setAngle(WristPositions.getFromArmvatorPosition(endPos).wristAngle.getRadians());
    }
  }
}
