package frc.robot.commands.NTControl;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.subsystems.arm.Pivot;

public class PivotNTControl extends Command {
  Pivot pivot;
  String NTPath = "Pivot/manualPosition";
  
  public PivotNTControl(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, Units.radiansToDegrees(pivot.getAngle().getRadians()));
    pivot.resetProfiledPidTo(pivot.getAngle());
    pivot.setMode(RunMode.PROFILED_PID);
  }

  @Override
  public void execute() {
    pivot.setMode(RunMode.PROFILED_PID);
    pivot.setGoalAngle(Units.degreesToRadians(SmartDashboard.getNumber(NTPath, pivot.getAngleRad())));
  }
}
