package frc.robot.commands.NTControl;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

public class WristNTControl extends Command {
  Wrist wrist;
  String NTPath = "Wrist/manualPosition";
  
  public WristNTControl(Wrist wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, Units.radiansToDegrees(wrist.getGoalAngle().getRadians()));
  }

  @Override
  public void execute() {
    wrist.setGoal(Units.degreesToRadians(SmartDashboard.getNumber(NTPath, wrist.getAngle().getRadians())));
  }
}
