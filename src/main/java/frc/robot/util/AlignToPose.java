package frc.robot.util;

import static frc.robot.constants.MiscConstants.*;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_DRIVE_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_ROT_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.DRIVE_CONTROLLER_PID;
import static frc.robot.constants.SwerveConstants.ROT_CONTROLLER_PID;
import static frc.robot.constants.VisionConstants.USE_2D_ALIGNMENT_MODE;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class AlignToPose {
  static ProfiledPIDController driveController = new ProfiledPIDController(DRIVE_CONTROLLER_PID[0], DRIVE_CONTROLLER_PID[1], DRIVE_CONTROLLER_PID[2], ALIGNMENT_DRIVE_CONSTRAINTS);
  static ProfiledPIDController rotController = new ProfiledPIDController(ROT_CONTROLLER_PID[0], ROT_CONTROLLER_PID[1], ROT_CONTROLLER_PID[2], ALIGNMENT_ROT_CONSTRAINTS);

  static {
    rotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /*
   * Pick objective based on:
   * whats in the end effector,
   * Operator input
   * 
   * Pick coral target based on:
   * Controller vector error to targets
   * distance to targets (translation and rotation)
   * 
   * Pick robot auto speed based on:
   * Magnitude of controller input
   * distance to target pose
   * (also the same controller vector?)
   */
  public static Pair<ChassisSpeeds, Double> getAutoAlignSpeeds(double controllerX, double controllerY, Pose2d robotPose, boolean L2Align) {
    Pose2d currentTarget = getCoralObjective(robotPose, controllerX, controllerY);
    if (L2Align) currentTarget = getL2Objective(robotPose, controllerX, controllerY);

    Logger.recordOutput("AlignToPose/currentTarget", currentTarget);

    double controllerMagnitude = MathUtil.clamp((Math.abs(controllerX) + Math.abs(controllerY)) / 2, 0, 0.33);
    Rotation2d rotationErrorToGoal = robotPose.getRotation().minus(currentTarget.getRotation());
    double targetDist = robotPose.getTranslation().getDistance(currentTarget.getTranslation());
    
    // calculate scaling factors
    // $$ y=8.6711x^{2}-5.89177x+1 $$
    // $$ \ce{H2O + H2O -> 2H2O}$$
    double controllerMagScalingFactor = (8.6711 * Math.pow((controllerMagnitude), 2) - 5.89177 * (controllerMagnitude) + 1);
    if (controllerMagnitude < 0.01) {
      controllerMagScalingFactor = 1;
    }
    Logger.recordOutput("AlignToPose/ControllerMagnitude", controllerMagnitude);
    double distScalingFactor = MathUtil.clamp(-1 * targetDist + 1, 0, 1);
    Logger.recordOutput("AlignToPose/ControllerScalingFactor", controllerMagScalingFactor);
    Logger.recordOutput("AlignToPose/DistanceScalingFactor", distScalingFactor);
    
    double totalScalingFactor = controllerMagScalingFactor * distScalingFactor;
    Logger.recordOutput("AlignToPose/TotalScalingFactor", totalScalingFactor);
    
    double driveScaledVel = driveController.calculate(targetDist, 0.0) * totalScalingFactor;
    Logger.recordOutput("AlignToPose/GoalAngleError", rotationErrorToGoal);
    Logger.recordOutput("AlignToPose/driveScaledVel", driveScaledVel);
    
    // actual drive to pose stuff, throw in reef avoidance getDriveTarget stuff
    Pose2d reefAvoidanceTarget = getReefAvoidanceTarget(robotPose, currentTarget);
    Rotation2d rotationErrorToCurrentTarget = robotPose.getRotation().minus(reefAvoidanceTarget.getRotation());
    Rotation2d translationVectorAngleError = robotPose.getTranslation().minus(reefAvoidanceTarget.getTranslation()).getAngle();
    Logger.recordOutput("AlignToPose/ReefAvoidanceTarget", reefAvoidanceTarget);
    var autoTranslation = new Pose2d(Translation2d.kZero,
      translationVectorAngleError)
      .transformBy(new Transform2d(new Translation2d(driveScaledVel, 0.0), Rotation2d.kZero))
      .getTranslation();

    double rotScaledVel = rotController.calculate(rotationErrorToCurrentTarget.getRadians(), 0.0) * totalScalingFactor;
    Logger.recordOutput("AlignToPose/rotScaledVel", rotScaledVel);

    ChassisSpeeds autoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(autoTranslation.getX(), autoTranslation.getY(), rotScaledVel, robotPose.getRotation());
    return Pair.of(autoSpeeds, totalScalingFactor);
  }

  // if you change any of the numbers in here auto will break
  public static Pose2d getReefAvoidanceTarget(Pose2d robot, Pose2d goal) {
    Pose2d offset = robot.relativeTo(goal);
    double xDist = Math.abs(offset.getX());
    double yDist = Math.abs(offset.getY());

    double shiftXT = MathUtil.clamp(
        (yDist / (REEF_FACE_LENGTH * 2)) + ((xDist - 0.3) / (REEF_FACE_LENGTH * 3)),
        0.0,
        1.0);
    double shiftYT = MathUtil.clamp(-MathUtil.clamp(offset.getX(), -MAX_REEF_LINEUP_DIST, 0) / REEF_FACE_LENGTH, 0.0,
        1.0);
    return goal.transformBy(
        new Transform2d(
            shiftXT * 1.5,
            Math.copySign(shiftYT * MAX_REEF_LINEUP_DIST * 1, offset.getY()), new Rotation2d()));
  }

  public static Pose2d getCoralObjective(Pose2d robotPose, double controllerX, double controllerY) {
    Translation2d controllerTranslation = new Translation2d(controllerX, controllerY);
    Rotation2d controllerVectorAngle = null;
    if (controllerTranslation.getNorm() > 0.02)
      controllerVectorAngle = controllerTranslation.getAngle();

    Pair<Pose2d, Double> bestReefPose = new Pair<Pose2d,Double>(null, 1000.0);
    ArrayList<Pose2d> reefPoses = new ArrayList<>(ReefPositions.getScoringPoses());
    // Logger.recordOutput("AlignToPose/reefPoses", Pose2d.struct, reefPoses.stream().toArray(Pose2d[]::new));
    for (int i = 0; i < reefPoses.size(); i++) {
      Pose2d reefPose = reefPoses.get(i);
      double dist = reefPose.getTranslation().getDistance(robotPose.getTranslation());

      double vectorError = 0;
      if (controllerVectorAngle != null)
        vectorError = Math.abs(controllerVectorAngle.minus(reefPose.getTranslation().getAngle()).getRotations());
      
      double weight = dist + (vectorError * VECTOR_ERROR_SCALAR);
      if (weight < bestReefPose.getSecond()) {
        bestReefPose = new Pair<Pose2d, Double>(reefPose, weight);
      }
    }
    return bestReefPose.getFirst();
  }

  public static Pose2d getL2Objective(Pose2d robotPose, double controllerX, double controllerY) {
    return getCoralObjective(robotPose, controllerX, controllerY).plus(new Transform2d(new Translation2d(0, 0), Rotation2d.k180deg));
  }

  public static Command getAutoAlignEndsCommand(Swerve swerve, Vision vision) {
    return swerve.run(() -> {
      var speeds = getAutoAlignSpeeds(0.0, 0.0, swerve.getPose(), false).getFirst();
      swerve.drive(speeds);
    }).until(() -> {
      return swerve.atPose(getCoralObjective(swerve.getPose(),0, 0));
    }).finallyDo(()->{
      vision.disableIdFiltering(1);
      vision.disableIdFiltering(2);
      vision.defaultPoseStrategies();
      vision.enableCamera(0);
    });
  }

  public static Command getAutoAlignCommand(Swerve swerve, Vision vision) {
    return swerve.run(() -> {
      var speeds = getAutoAlignSpeeds(0.0, 0.0, swerve.getPose(), false).getFirst();
      swerve.drive(speeds);
    }).finallyDo(()->{
      vision.disableIdFiltering(1);
      vision.disableIdFiltering(2);
      vision.defaultPoseStrategies();
      vision.enableCamera(0);
    });
  }

  public static void enableReefVision(Vision vision, Rotation2d gyroOffset, int nearestTagId) {
    if (USE_2D_ALIGNMENT_MODE) {
      vision.setGyroOffset(gyroOffset);
      vision.setCameraPreference(1);
      vision.setPoseStrategy(1, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
      vision.setPoseStrategy(2, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
      vision.filterTagById(1, nearestTagId);
      vision.filterTagById(2, nearestTagId);
      vision.disableCamera(0);
    }
  }

  public static void disableReefVision(Vision vision) {
    vision.disableIdFiltering(1);
    vision.disableIdFiltering(2);
    vision.defaultPoseStrategies();
    vision.enableCamera(0);
  }
}
