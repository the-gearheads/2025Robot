package frc.robot.util;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Swerve;

public class ObjectiveTracker {
  double lookAheadS = 0.15;
  Swerve swerve;
  Pose2d predictedRobot;
  AprilTagFieldLayout layout;

  public ObjectiveTracker(Swerve swerve) {
    this.swerve = swerve;
  }

  public void periodic() {
  }


  public Pose2d getCoralObjective() {
    predictedRobot = swerve.getPose().exp(
        swerve.getRobotRelativeSpeeds().toTwist2d(lookAheadS));
    Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(22).get().toPose2d();
    Logger.recordOutput("ObjectiveTracker/PredictedPose", predictedRobot);
    Logger.recordOutput("ObjectiveTracker/TagPose", tagPose);
    Pose2d errorPose = predictedRobot
        .relativeTo(tagPose);
    Logger.recordOutput("ObjectiveTracker/errorPose", errorPose);
    return new Pose2d();

  }
}
