package frc.robot.commands;


import static frc.robot.constants.ArmConstants.MAX_ANGLE;
import static frc.robot.constants.ArmConstants.MIN_ANGLE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

  double targetAngleRad = Math.PI / 2.0;

  @Override
  public void initialize() {
    pivot.setMode(RunMode.PROFILED_PID);
    targetAngleRad = pivot.getAngle().getRadians();
    pivot.setGoalAngle(targetAngleRad);
    pivot.resetProfiledPidTo(Rotation2d.fromRadians(targetAngleRad));

  }

  @Override
  public void execute() {
    pivot.setGoalAngle(targetAngleRad);
    
    double speed = Units.inchesToMeters(0);
    if (Controllers.driverController.getXBtn()) {
      speed = -1;
    }
    if (Controllers.driverController.getABtn()) {
      speed = 1;
    }
    targetAngleRad += (speed * 0.003);
    targetAngleRad = MathUtil.clamp(targetAngleRad, MIN_ANGLE, MAX_ANGLE);
    pivot.setGoalAngle(targetAngleRad);


  }
}
