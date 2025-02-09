package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.constants.ArmConstants.ELEVATOR_MASS;
import static frc.robot.constants.ArmConstants.MAX_HEIGHT;
import static frc.robot.constants.ArmConstants.MIN_HEIGHT;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class telescopeSim extends Telescope {
  DCMotor elevatorMotor = DCMotor.getNeoVortex(2);
  ElevatorSim teleSim = new ElevatorSim(LinearSystemId.createElevatorSystem(elevatorMotor, ELEVATOR_MASS, 0.15, ELEVATOR_GEAR_RATIO), elevatorMotor, MIN_HEIGHT, MAX_HEIGHT, true, MIN_HEIGHT);
  SparkFlexSim elevatorFlexSim = new SparkFlexSim(elevator, elevatorMotor);

  public telescopeSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {
    var out = elevatorFlexSim.getAppliedOutput();
    out = 1;
    teleSim.setInputVoltage(out * RoboRioSim.getVInVoltage());
    teleSim.update(0.02);
    elevatorFlexSim.iterate(teleSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02); // might not work theres a separate iterate for abs encoder that might need to be called????
  }
}
