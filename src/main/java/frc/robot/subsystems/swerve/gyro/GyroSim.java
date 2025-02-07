package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GyroSim implements Gyro {

  double yaw;
  double velocityYaw;

  public GyroSim() {
  }

  public void reset() {
    yaw = 0;
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(yaw);
  }

  public Rotation3d getRotation3d() {
    return new Rotation3d(0, 0, yaw);
  }

  // Sets sim angle, rad.
  public void setYaw(double yaw) {
    this.yaw = yaw;
  }

  public double getYaw() {
    return yaw;
  }

  public double getVelocityYaw() {
    return velocityYaw;
  }

  // Sets sim velocity, rad/sec.
  public double setVelocityYaw(double velocity) {
    return velocityYaw = velocity;
  }
}
