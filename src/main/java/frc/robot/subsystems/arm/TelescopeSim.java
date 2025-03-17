package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class TelescopeSim extends Telescope {
  DCMotor elevatorMotor = DCMotor.getNeoVortex(2);
  ElevatorSim teleSim = new ElevatorSim(LinearSystemId.identifyPositionSystem(ELEVATOR_KV, ELEVATOR_KA), elevatorMotor, MIN_RELATIVE_HEIGHT, MAX_RELATIVE_HEIGHT, false, MIN_RELATIVE_HEIGHT );

  public TelescopeSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {
    teleSim.update(0.02);
  }

  @Override
  public double getExtension() {
    if (teleSim == null) {
      return 0;
    }
    return teleSim.getPositionMeters();
  }

  @Override
  protected void setMotorVoltage(double volts) {
    volts = MathUtil.clamp(volts, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    teleSim.setInputVoltage(volts);
  }

  @Override
  public double getVelocity() {
    if (teleSim == null) {
      return 0;
    }
    return teleSim.getVelocityMetersPerSecond();
  }

  @Override
  public boolean getLimitSwitch() {
    return getExtension() < 0.051;
  }

}
