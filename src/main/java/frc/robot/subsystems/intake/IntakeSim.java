package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.INTAKE_GEAR_RATIO;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeSim extends Intake {
  FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 1, INTAKE_GEAR_RATIO), DCMotor.getNeo550(1), INTAKE_GEAR_RATIO);

  @AutoLogOutput
  public double getVelocity() {
    return intakeSim.getAngularVelocityRPM();
  }

  @Override
  public void simulationPeriodic() {
    intakeSim.update(0.02);
  }

  @Override
  public void setMotorVoltage(double volts) {
    volts = MathUtil.clamp(volts, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    intakeSim.setInputVoltage(volts);
  }
}
