package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmvatorPosition;

public enum SuperstructurePosition {
  HP(ArmvatorPosition.HP, Units.degreesToRadians(43)),  // good
  L1(ArmvatorPosition.L1, Units.degreesToRadians(-7 - 30)),
  L2(ArmvatorPosition.L2, Units.degreesToRadians(120)), // good
  L3(ArmvatorPosition.L3, Units.degreesToRadians(-36.0)), // good
  L4(ArmvatorPosition.L4, Units.degreesToRadians(-96.13882298)), // good
  NET(ArmvatorPosition.NET, Units.degreesToRadians(40
  )), // good
  GROUND_INTAKE(ArmvatorPosition.GROUND_INTAKE, Units.degreesToRadians(110)),  // good
  PROCESSOR(ArmvatorPosition.PROCESSOR, Units.degreesToRadians(-10));

  public final ArmvatorPosition armvatorPosition;
  public final Rotation2d wristAngle;

  private SuperstructurePosition(ArmvatorPosition pos, double wristAngle) {
    this.armvatorPosition = pos;
    this.wristAngle = Rotation2d.fromRadians(wristAngle);
  }
}
