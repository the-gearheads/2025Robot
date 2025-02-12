package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.ELEVATOR_FEEDFORWARD;
import static frc.robot.constants.ArmConstants.PIVOT_FEEDFORWARD;

import frc.robot.Robot;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class SuperStructure {
  public final Pivot pivot;
  public final Telescope telescope;

  private double ffPivot;
  private double outputPivot;
  private double ffTelescope;
  private double outputTelescope;

  public SuperStructure() {
    if (Robot.isReal()) {
      pivot = new Pivot();
      telescope = new Telescope();
    } else {
      pivot = new PivotSim();
      telescope = new telescopeSim();
    }
  }

  public void followSample(ArmvatorSample sample) {
    ffPivot = PIVOT_FEEDFORWARD.calculate(sample.armPos(), sample.armVel(), sample.armAccel());
    outputPivot = pivot.pid.calculate(pivot.getAngle().getRadians(), sample.armPos()) + ffPivot;
    pivot.setVoltage(outputPivot);
    
    ffTelescope = ELEVATOR_FEEDFORWARD.calculate(sample.elevatorVel(), sample.elevatorAccel());
    outputTelescope = telescope.elevatorPid.calculate(telescope.getPosition(), sample.elevatorLen());
    telescope.setVoltage(outputTelescope);
  }

  public void followTrajectory(ArmvatorTrajectory traj) {

  }
}
