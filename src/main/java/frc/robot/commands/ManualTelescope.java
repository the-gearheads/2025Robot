package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.SuperStructure.RunMode;

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
    double voltage = 12;
    voltage = Controllers.driverController.getSpeedUpAxis() * 12;
    voltage -= Controllers.driverController.getSlowDownAxis() * 12;
    telescope.setVoltage(voltage);
  }
}
