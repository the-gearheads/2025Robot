package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SysidAutoPicker {
  private SendableChooser<Command> chooser = new SendableChooser<>();

  public SysidAutoPicker() {
    SmartDashboard.putData("SysidRoutines", chooser);
  }

  public void addSysidRoutine(Command routine, String name) {
    chooser.addOption(name, routine);
    chooser.addOption(name, routine);
    chooser.addOption(name, routine);
    chooser.addOption(name, routine);
  }

  public Command get() {
    return chooser.getSelected();
  }
}
