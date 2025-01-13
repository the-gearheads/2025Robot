package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase{

  public Pose2d getPose() {
    return new Pose2d();
  }

  public Pose2d getPoseWheelsOnly() {
    return new Pose2d();
  }
}
