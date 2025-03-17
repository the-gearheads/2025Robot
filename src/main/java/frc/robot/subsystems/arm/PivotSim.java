package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotSim extends Pivot {
  DCMotor pivotMotorSim = DCMotor.getNeoVortex(2);
  SingleJointedArmSim armSim = new SingleJointedArmSim(LinearSystemId.identifyPositionSystem(PIVOT_KV, PIVOT_KA), pivotMotorSim, PIVOT_GEAR_RATIO, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, false, 15);

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
  public Rotation2d getAbsAngle() {
    return getAngle();
  }

  @Override
  public double getVelocity() {
    return armSim.getVelocityRadPerSec();
  }

  @Override
  public void setMotorVoltage(double volts) {
    // Gravity sim aint realistic for us so turn it off and subtraect the ff effects on it. this probably inadvertently also simulates gravity lmao
    volts -= PIVOT_KS - Math.signum(volts) * PIVOT_KG * Math.cos(getAngle().getRadians());
    volts = MathUtil.clamp(volts, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    armSim.setInputVoltage(volts);
  }


}
