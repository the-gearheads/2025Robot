package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GyroRedux implements Gyro {
  Canandgyro gyro = new Canandgyro(1);
  CanandgyroSettings settings = new CanandgyroSettings();
  public GyroRedux() {
    settings.setEphemeral(true);
    gyro.setSettings(settings);
  }

  public void reset() {
    gyro.setYaw(0);
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public Rotation3d getRotation3d() {
    return gyro.getRotation3d();
  }

  public double getYaw() {
    return gyro.getMultiturnYaw() * (2 * Math.PI); // rot -> rad
  }

  public double getVelocityYaw() {
    return gyro.getAngularVelocityYaw() * (2 * Math.PI); // rot/sec -> rad/sec
  }

  public void log() {
    Logger.recordOutput("Swerve/GyroRedux/Faults", gyro.getActiveFaults());
    Logger.recordOutput("Swerve/GyroRedux/StickyFaults", gyro.getStickyFaults());
    Logger.recordOutput("Swerve/GyroRedux/IsConnected", gyro.isConnected());
  }
}
