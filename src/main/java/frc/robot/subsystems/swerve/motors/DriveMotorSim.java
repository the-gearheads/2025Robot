package frc.robot.subsystems.swerve.motors;

import static frc.robot.constants.SwerveConstants.DRIVE_FEEDFORWARD;
import static frc.robot.constants.SwerveConstants.DRIVE_PID;
import static frc.robot.constants.SwerveConstants.DRIVE_RATIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.OptionalDouble;

public class DriveMotorSim extends DriveMotor {

  FlywheelSim sim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(DRIVE_FEEDFORWARD.getKv(), DRIVE_FEEDFORWARD.getKa()), DCMotor.getNeoVortex(1).withReduction(DRIVE_RATIO));
  PIDController softPid = new PIDController(DRIVE_PID[0], DRIVE_PID[1], DRIVE_PID[2]);

  double pos = 0;
  double volts = 0;

  public DriveMotorSim(int id, int index, String modulePath) {
    super(id, index, modulePath);
  }

  // i think we need to override periodic, setVoltage, getPosition, resetEncoder, and getVelocity

  @Override
  public void periodic() {
    super.periodic();
    sim.update(0.02);
    setVoltage(softPid.calculate(sim.getAngularVelocityRPM(), driveSetpoint) + DRIVE_FEEDFORWARD.calculate(driveSetpoint));
    pos += sim.getAngularVelocityRadPerSec() * 0.02;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    double batteryVolts = RobotController.getBatteryVoltage();
    sim.setInputVoltage(MathUtil.clamp(volts, -batteryVolts, batteryVolts));
  }
  
  @Override
  public double getVoltage() {
    return volts;
  }

  @Override
  public double getPosition() {
    return pos;
  }

  @Override
  public OptionalDouble getPositionOptional() {
    return OptionalDouble.of(pos);
  }

  @Override
  public void resetEncoder() {
    pos = 0;
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }
}
