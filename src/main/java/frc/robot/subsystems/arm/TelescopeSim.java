package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.constants.ArmConstants.ELEVATOR_MASS;
import static frc.robot.constants.ArmConstants.MAX_RELATIVE_HEIGHT;
import static frc.robot.constants.ArmConstants.MIN_RELATIVE_HEIGHT;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class TelescopeSim extends Telescope {
  DCMotor elevatorMotor = DCMotor.getNeoVortex(2);
  ElevatorSim teleSim = new ElevatorSim(LinearSystemId.createElevatorSystem(elevatorMotor, ELEVATOR_MASS, 0.15, ELEVATOR_GEAR_RATIO), elevatorMotor, MIN_RELATIVE_HEIGHT, MAX_RELATIVE_HEIGHT, false, MIN_RELATIVE_HEIGHT);
  double output = 0;

  public TelescopeSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {
    teleSim.update(0.02);
  }

  @Override
  public double getPosition() {
    if (teleSim == null) {
      return 0;
    }
    return teleSim.getPositionMeters();
  }

  @Override
  protected void setMotorVoltage(double volts) {
    teleSim.setInputVoltage(volts);
    output = volts;
  }

  @Override
  public double getVelocity() {
    if (teleSim == null) {
      return 0;
    }
    return teleSim.getVelocityMetersPerSecond();
  }

}
