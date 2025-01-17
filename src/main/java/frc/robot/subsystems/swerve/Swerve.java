package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase{
  
  static final Lock odometryLock = new ReentrantLock();
  
  public Pose2d getPose() {
    return new Pose2d();
  }

  public Pose2d getPoseWheelsOnly() {
    return new Pose2d();
  }
}
