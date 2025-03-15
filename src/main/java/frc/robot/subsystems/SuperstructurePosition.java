package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmvatorPosition;

public enum SuperstructurePosition {
  HP(ArmvatorPosition.HP, Units.degreesToRadians(43)),  // new good
  L1(ArmvatorPosition.L1, Units.degreesToRadians(121.9347)), // NEW GOOD
  L2(ArmvatorPosition.L2, Units.degreesToRadians(125.585)), // new good
  L3(ArmvatorPosition.L3, Units.degreesToRadians(-36.0)), // new good
  L4(ArmvatorPosition.L4, Units.degreesToRadians(-62.614)), // new good
  NET(ArmvatorPosition.NET, Units.degreesToRadians(40)), // untested
  GROUND_INTAKE(ArmvatorPosition.GROUND_INTAKE, Units.degreesToRadians(110)), 
  PROCESSOR(ArmvatorPosition.PROCESSOR, Units.degreesToRadians(-10));

  public final ArmvatorPosition armvatorPosition;
  public final Rotation2d wristAngle;

  private SuperstructurePosition(ArmvatorPosition pos, double wristAngle) {
    this.armvatorPosition = pos;
    this.wristAngle = Rotation2d.fromRadians(wristAngle);
  }
}
