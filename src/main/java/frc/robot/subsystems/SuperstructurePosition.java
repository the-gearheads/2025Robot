package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmvatorPosition;

public enum SuperstructurePosition {
  HP(ArmvatorPosition.HP, Units.degreesToRadians(43)), 
  L1(ArmvatorPosition.L1, Units.degreesToRadians(121.9347)),
  L2(ArmvatorPosition.L2, Units.degreesToRadians(125.585)),
  L3(ArmvatorPosition.L3, Units.degreesToRadians(-36.0)),
  L4(ArmvatorPosition.L4, Units.degreesToRadians(-48.71)),
  AlgaeL3(ArmvatorPosition.AlgaeL3, Units.degreesToRadians(0)),
  AlgaeL2(ArmvatorPosition.AlgaeL2, Units.degreesToRadians(124.36)),
  NET(ArmvatorPosition.NET, Units.degreesToRadians(40)),
  GROUND_INTAKE(ArmvatorPosition.GROUND_INTAKE, Units.degreesToRadians(110)), 
  PROCESSOR(ArmvatorPosition.PROCESSOR, Units.degreesToRadians(115.55)),
  STOW(ArmvatorPosition.STOW, Units.degreesToRadians(-30));

  public final ArmvatorPosition armvatorPosition;
  public final Rotation2d wristAngle;

  private SuperstructurePosition(ArmvatorPosition pos, double wristAngle) {
    this.armvatorPosition = pos;
    this.wristAngle = Rotation2d.fromRadians(wristAngle);
  }
}
