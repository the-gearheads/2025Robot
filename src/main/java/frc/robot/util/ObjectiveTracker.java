package frc.robot.util;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    Pose2d alignmentPose;
    Transform2d errorPose;
    Pose2d currentPose = new Pose2d();
    double minAngle = 10;
    double minDist = 2;  // m
    predictedRobot = swerve.getPose().exp(
        swerve.getRobotRelativeSpeeds().toTwist2d(lookAheadS));
    Logger.recordOutput("ObjectiveTracker/PredictedPose", predictedRobot);
    Logger.recordOutput("ObjectiveTracker/Nearest", predictedRobot.nearest(ReefPositions.getReefPoses()));
    // Pose2d[] test = new Pose2d[8];
    // for (int i = 0; i < 8; i++) {
    //   // -1 left, 1 right
    //   alignmentPose = ReefPositions.getReefPose(i, -1);
      // test[i] = alignmentPose;
      // Logger.recordOutput("ObjectiveTracker/AlignmentPose", alignmentPose);
      // errorPose = alignmentPose.minus(predictedRobot);
      // alignmentPose.nearest(predictedRobot);
      // Logger.recordOutput("ObjectiveTracker/ErrorPose", errorPose);
      // // Logger.recordOutput("ObjectiveTracker/Error+Pred", errorPose.minus(predictedRobot));
      // if (Math.abs(errorPose.getTranslation().getNorm()) < minDist) {
      //   minDist = errorPose.getX() + errorPose.getY();
      //   minAngle = errorPose.getRotation().getRadians();
      //   currentPose = alignmentPose;
      // }
      // alignmentPose = ReefPositions.getReefPose(i, 1);
      // Logger.recordOutput("ObjectiveTracker/AlignmentPose", alignmentPose);
      // errorPose = alignmentPose.relativeTo(predictedRobot);
      // Logger.recordOutput("ObjectiveTracker/ErrorPose", errorPose);
      // Logger.recordOutput("ObjectiveTracker/Error+Pred", errorPose.plus(new Transform2d(predictedRobot.getTranslation(), predictedRobot.getRotation())));
      // if (errorPose.getTranslation().getNorm() < minDist) {
      //   if (errorPose.getRotation().getRadians() < minAngle) {
      //     minDist = errorPose.getX() + errorPose.getY();
      //     minAngle = errorPose.getRotation().getRadians();
      //     currentPose = alignmentPose;
      //   }
      // }
    // }
    // Logger.recordOutput("ObjectiveTracker/test", test);
    // Logger.recordOutput("ObjectiveTracker/SelectedPose", currentPose);
    return currentPose;
  }

// 1/2 m to 20 deg
}
