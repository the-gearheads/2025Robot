package frc.robot.commands;

import static frc.robot.constants.ArmConstants.MAX_RELATIVE_HEIGHT;
import static frc.robot.constants.ArmConstants.MIN_RELATIVE_HEIGHT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.subsystems.arm.Telescope;

public class ManualTelescope extends Command {
  Telescope telescope;
  public ManualTelescope(Telescope telescope) {
    this.telescope = telescope;
    addRequirements(telescope);
  }

  double targetPos = MIN_RELATIVE_HEIGHT;

  @Override
  public void initialize() {
    telescope.setMode(RunMode.PROFILED_PID);
    targetPos = telescope.getLength();
    telescope.setGoalPosition(targetPos);
    telescope.resetProfiledPidTo(targetPos);
  }

  @Override
  public void execute() {
    double speed = Units.inchesToMeters(4);
    speed = Controllers.driverController.getSpeedUpAxis() * 4;
    speed -= Controllers.driverController.getSlowDownAxis() * 4;
    targetPos += (speed * 0.02);
    targetPos = MathUtil.clamp(targetPos, MIN_RELATIVE_HEIGHT, MAX_RELATIVE_HEIGHT);
    telescope.setGoalPosition(targetPos);
  }
}
