package frc.robot.commands;

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

  @Override
  public void initialize() {
    telescope.setMode(RunMode.VOLTAGE);
  }

  @Override
  public void execute() {
    double voltage = 2;
    voltage = Controllers.driverController.getSpeedUpAxis() * 2;
    voltage -= Controllers.driverController.getSlowDownAxis() * 2;
    telescope.setVoltage(voltage);
  }
}
