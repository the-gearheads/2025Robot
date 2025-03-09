package frc.robot.subsystems.wrist;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristSim extends Wrist {
  public WristSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(pid.getSetpoint().position);
  }

  @Override
  public double getVelocity() {
    return pid.getSetpoint().velocity;
  }

  @Override
  public void setMotorVoltage(double volts) {
    output = volts;
  }
}
