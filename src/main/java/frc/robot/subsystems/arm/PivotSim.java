package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.ARM_LENGTH;
import static frc.robot.constants.ArmConstants.MAX_ANGLE;
import static frc.robot.constants.ArmConstants.MIN_ANGLE;
import static frc.robot.constants.ArmConstants.PIVOT_GEAR_RATIO;
import static frc.robot.constants.ArmConstants.PIVOT_MOI_EST;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotSim extends Pivot {
  DCMotor pivotMotorSim = DCMotor.getNeoVortex(2);
  SingleJointedArmSim armSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(pivotMotorSim, PIVOT_MOI_EST, PIVOT_GEAR_RATIO), pivotMotorSim, PIVOT_GEAR_RATIO, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, true, 15);
  double output;

  public PivotSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {
    armSim.update(0.02);
  }

  @Override
  public Rotation2d getAngle() {
    if (armSim == null) return new Rotation2d();
    return Rotation2d.fromRadians(armSim.getAngleRads());
  }

  @Override
  public double getVelocity() {
    return armSim.getVelocityRadPerSec();
  }

  @Override
  public void setMotorVoltage(double volts) {
    armSim.setInputVoltage(volts);
    output = volts;
  }


}
