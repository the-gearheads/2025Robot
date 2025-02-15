package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.SuperStructure.RunMode;

public class ManualPivot extends Command {
  Pivot pivot;

  public ManualPivot(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setMode(RunMode.VOLTAGE);
  }
  
  @Override
  public void execute() {
    if (Controllers.driverController.getXBtn()) {
      pivot.setVoltage(-6);
    } else if (Controllers.driverController.getABtn()) {
      pivot.setVoltage(6);
    } else {
      pivot.setVoltage(0);
    }
  }
}
