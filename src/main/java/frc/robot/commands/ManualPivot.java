package frc.robot.commands;


import static frc.robot.constants.ArmConstants.MAX_ANGLE;
import static frc.robot.constants.ArmConstants.MIN_ANGLE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.subsystems.arm.Pivot;

public class ManualPivot extends Command {
  Pivot pivot;
  public ManualPivot(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  Rotation2d targetAngle = Rotation2d.kCW_90deg;

  @Override
  public void initialize() {
    pivot.setMode(RunMode.PROFILED_PID);
    targetAngle = pivot.getAngle();
    pivot.setGoalAngle(targetAngle);
    pivot.resetProfiledPidTo(targetAngle);
  }

  @Override
  public void execute() {
    targetAngle = pivot.getAngle();
    if (Controllers.driverController.getXBtn()) {
      targetAngle = targetAngle.plus(Rotation2d.fromDegrees(2));
    }
    if (Controllers.driverController.getABtn()) {
      targetAngle = targetAngle.minus(Rotation2d.fromDegrees(2));
    }

    double targetAngleRad = MathUtil.clamp(targetAngle.getRadians(), MIN_ANGLE, MAX_ANGLE);
    pivot.setGoalAngle(targetAngleRad);
  }
}
