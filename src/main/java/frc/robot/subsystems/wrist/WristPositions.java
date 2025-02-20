package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmvatorPosition;

public enum WristPositions {
  HP(ArmvatorPosition.HP, Units.degreesToRadians(-5)),
  L1(ArmvatorPosition.L1, Units.degreesToRadians(-7)),
  L2(ArmvatorPosition.L2, Units.degreesToRadians(-10)),
  L3(ArmvatorPosition.L3, Units.degreesToRadians(115)),
  L4(ArmvatorPosition.L4, Units.degreesToRadians(150)),
  NET(ArmvatorPosition.NET, Units.degreesToRadians(20)),
  GROUND_INTAKE(ArmvatorPosition.GROUND_INTAKE, Units.degreesToRadians(-20)),
  PROCESSOR(ArmvatorPosition.PROCESSOR, Units.degreesToRadians(-10));

  private final ArmvatorPosition correspondingArmvatorPosition;
  public final Rotation2d wristAngle;

  private WristPositions(ArmvatorPosition pos, double wristAngle) {
    correspondingArmvatorPosition = pos;
    this.wristAngle = Rotation2d.fromRadians(wristAngle);
  }

  public static WristPositions getFromArmvatorPosition(ArmvatorPosition pos) {
    for (var position : WristPositions.values()) {
      if (position.correspondingArmvatorPosition == pos) {
        return position;
      }
    }
    return WristPositions.L1;
  }
}
