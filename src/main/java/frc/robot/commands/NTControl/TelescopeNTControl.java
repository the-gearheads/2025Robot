package frc.robot.commands.NTControl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.subsystems.arm.Telescope;

public class TelescopeNTControl extends Command {
  Telescope telescope;
  String NTPath = "Telescope/manualPosition";
  
  public TelescopeNTControl(Telescope telescope) {
    this.telescope = telescope;
    addRequirements(telescope);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, telescope.getExtension());
    telescope.setMode(RunMode.PROFILED_PID);
  }

  @Override
  public void execute() {
    telescope.setMode(RunMode.PROFILED_PID);
    telescope.setGoalPosition(SmartDashboard.getNumber(NTPath, telescope.getExtension()));
  }
}
