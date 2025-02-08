package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLogOutput;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroNavx implements Gyro {
  
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public GyroNavx() {}
  
    public void reset() {
      gyro.reset();
      gyro.resetDisplacement();
    }
  
    @AutoLogOutput(key="Swerve/GyroNavx/rotation2d")
    public Rotation2d getRotation2d() {
      return gyro.getRotation2d();
    }

    @AutoLogOutput(key="Swerve/GyroNavx/rotation3d")
    public Rotation3d getRotation3d() {
      return gyro.getRotation3d();
    }

    @AutoLogOutput(key="Swerve/GyroNavx/velocityYaw")
    public double getVelocityYaw() {
      return Units.degreesToRadians(-gyro.getRate()); // while getRotation2d inverts for us, all other funcs don't
    }

    @AutoLogOutput(key="Swerve/GyroNavx/yaw")
    public double getYaw() {
      return Units.degreesToRadians(-gyro.getAngle());
    }

    @AutoLogOutput(key="Swerve/GyroNavx/isConnected")
    public boolean isConnected() {
      return gyro.isConnected();
    }

}
