package frc.robot.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroNavx implements Gyro {
  
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public GyroNavx() {

    }
  
    public void reset() {
      gyro.reset();
    }
  
    public Rotation2d getRotation2d() {
      return gyro.getRotation2d();
    }

    public Rotation3d getRotation3d() {
      return gyro.getRotation3d();
    }

    public double getVelocityYaw() {
      return Units.degreesToRadians(-gyro.getRate()); // while getRotation2d inverts for us, all other funcs don't
    }

    public double getYaw() {
      return Units.degreesToRadians(-gyro.getAngle());
    }

    public boolean isConnected() {
      return gyro.isConnected();
    }
}
