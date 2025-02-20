package frc.robot.subsystems.wrist;

import static frc.robot.constants.WristConstants.MAX_WRIST_ANGLE;
import static frc.robot.constants.WristConstants.MIN_WRIST_ANGLE;
import static frc.robot.constants.WristConstants.WRIST_GEAR_RATIO;
import static frc.robot.constants.WristConstants.WRIST_LENGTH;
import static frc.robot.constants.WristConstants.WRIST_MOI_EST;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristSim extends Wrist {
  DCMotor wristMotorSim = DCMotor.getNeo550(1);
  SingleJointedArmSim wristSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(wristMotorSim, WRIST_MOI_EST, WRIST_GEAR_RATIO), wristMotorSim, WRIST_GEAR_RATIO, WRIST_LENGTH, MIN_WRIST_ANGLE, MAX_WRIST_ANGLE, false, 1);

  double output;
  public WristSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {
    wristSim.update(0.02);
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(wristSim.getAngleRads());
  }

  @Override
  public double getVelocity() {
    return wristSim.getVelocityRadPerSec();
  }

  @Override
  public void setMotorVoltage(double volts) {
    wristSim.setInput(volts);
    output = volts;
  }
}
