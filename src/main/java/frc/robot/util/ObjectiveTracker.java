package frc.robot.util;

import static frc.robot.constants.MiscConstants.AUTO_ALIGN_FILTER_ANGLE;
import static java.util.stream.Collectors.toList;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.Swerve;

public class ObjectiveTracker {
  double lookAheadS = 0.15;
  Swerve swerve;
  Pose2d predictedRobot;
  AprilTagFieldLayout layout;
  Pose2d selectedPose;

  public ObjectiveTracker(Swerve swerve) {
    this.swerve = swerve;
  }

  public Pose2d getCoralObjective(Rotation2d controllerVectorAngle) {
    predictedRobot = swerve.getPose().exp(
        swerve.getRobotRelativeSpeeds().toTwist2d(lookAheadS));
    // filter poses by angle
    List<Pose2d> withinAnglePoses = ReefPositions.getScoringPoses().stream()
        .filter(pose -> Math.abs(
            pose.getRotation().getRadians() - predictedRobot.getRotation().getRadians()) <= AUTO_ALIGN_FILTER_ANGLE)
        .collect(toList());

    if (withinAnglePoses.isEmpty()) {
      selectedPose = null;
    } else {
      selectedPose = predictedRobot.nearest(withinAnglePoses);
    }

    Logger.recordOutput("ObjectiveTracker/PredictedPose", predictedRobot);
    Logger.recordOutput("ObjectiveTracker/NearestPose", predictedRobot.nearest(ReefPositions.getScoringPoses()));
    Logger.recordOutput("ObjectiveTracker/SelectedPose", selectedPose);
    return selectedPose;
  }

  public boolean facingReef() {
    Pose2d currentPose = swerve.getPose();
    Pose2d closestReefPose = currentPose.nearest(ReefPositions.getCenterPoses());
    double angleError = Math.abs(currentPose.getRotation().minus(closestReefPose.getRotation()).getRadians());

    if (angleError < Math.PI / 2.0) {
      return false;
    }
    return true;
  } 
}
