package frc.robot.commands;

import static frc.robot.constants.MiscConstants.*;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_DRIVE_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_ROT_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_ROT_SPEED;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;
import static frc.robot.constants.SwerveConstants.XY_PATH_FOLLOWING_PID;
import static frc.robot.constants.SwerveConstants.ROT_PATH_FOLLOWING_PID;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ReefPositions;

public class AlignToPose extends Command {
  Swerve swerve;
  ProfiledPIDController driveController = new ProfiledPIDController(XY_PATH_FOLLOWING_PID[0], XY_PATH_FOLLOWING_PID[1],
      XY_PATH_FOLLOWING_PID[2], ALIGNMENT_DRIVE_CONSTRAINTS);
  ProfiledPIDController rotController = new ProfiledPIDController(ROT_PATH_FOLLOWING_PID[0], ROT_PATH_FOLLOWING_PID[1],
      ROT_PATH_FOLLOWING_PID[2], ALIGNMENT_ROT_CONSTRAINTS);

  LinearFilter controllerXFilter = LinearFilter.highPass(0.1, 0.02);
  LinearFilter controllerYFilter = LinearFilter.highPass(0.1, 0.02);

  public AlignToPose(Swerve swerve) {
    this.swerve = swerve;
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
    SmartDashboard.putData("AlignToPose/rotController", rotController);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerve.getPose();
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    Translation2d controllerTranslation = new Translation2d(x, y);
    Rotation2d controllerAngle = controllerTranslation.getAngle();

    driveController.reset(
        currentPose.getTranslation().getDistance(getCoralObjective(controllerAngle).getTranslation()),
        Math.min(0,
            -new Translation2d(
                ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
                    swerve.getPose().getRotation()).vxMetersPerSecond,
                ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
                    swerve.getPose().getRotation()).vyMetersPerSecond)
                .rotateBy(
                  getCoralObjective(controllerAngle)
                        .getTranslation()
                        .minus(swerve.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));


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
  @Override
  public void execute() {
    SmartDashboard.putData("AlignToPose/rotController", rotController);
    // controller
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    Translation2d controllerTranslation = new Translation2d(x, y);
    Rotation2d controllerAngle = controllerTranslation.getAngle();

    // poses
    Pose2d currentPose = swerve.getPose();
    Pose2d currentTarget = getCoralObjective(controllerAngle);
    Logger.recordOutput("AlignToPose/currentTarget", currentTarget);

    double controllerMagnitude = MathUtil.clamp((Math.abs(x) + Math.abs(y)) / 2, 0, 0.33);
    Rotation2d rotationErrorToGoal = currentPose.getRotation().minus(currentTarget.getRotation());
    double targetDist = currentPose.getTranslation().getDistance(currentTarget.getTranslation());
    
    // calculate scaling factors
    // $$ y=8.6711x^{2}-5.89177x+1 $$
    // $$ \ce{H2O + H2O -> 2H2O}$$
    double controllerMagScalingFactor = (8.6711 * Math.pow((controllerMagnitude), 2) - 5.89177 * (controllerMagnitude) + 1);
    if (controllerMagnitude < 0.01) {
      controllerMagScalingFactor = 1;
    }
    Logger.recordOutput("AlignToPose/ControllerMagnitude", controllerMagnitude);
    double distScalingFactor = MathUtil.clamp(-0.7679 * targetDist + 1, 0, 1);
    Logger.recordOutput("AlignToPose/ControllerScalingFactor", controllerMagScalingFactor);
    Logger.recordOutput("AlignToPose/DistanceScalingFactor", distScalingFactor);
    
    double totalScalingFactor = controllerMagScalingFactor * distScalingFactor;
    Logger.recordOutput("AlignToPose/TotalScalingFactor", totalScalingFactor);
    
    double driveScaledVel = driveController.calculate(targetDist, 0.0) * totalScalingFactor;
    Logger.recordOutput("AlignToPose/GoalAngleError", rotationErrorToGoal);
    Logger.recordOutput("AlignToPose/driveScaledVel", driveScaledVel);
    
    // actual drive to pose stuff, throw in reef avoidance getDriveTarget stuff
    Pose2d reefAvoidanceTarget = getDriveTarget(currentPose, currentTarget);
    Rotation2d rotationErrorToCurrentTarget = currentPose.getRotation().minus(reefAvoidanceTarget.getRotation());
    Rotation2d translationVectorAngleError = currentPose.getTranslation().minus(reefAvoidanceTarget.getTranslation()).getAngle();
    Logger.recordOutput("AlignToPose/ReefAvoidanceTarget", reefAvoidanceTarget);
    var autoTranslation = new Pose2d(Translation2d.kZero,
      translationVectorAngleError)
      .transformBy(new Transform2d(new Translation2d(driveScaledVel, 0.0), Rotation2d.kZero))
      .getTranslation();

    double rotScaledVel = rotController.calculate(rotationErrorToCurrentTarget.getRadians(), 0.0) * totalScalingFactor;
    Logger.recordOutput("AlignToPose/rotScaledVel", rotScaledVel);

    double xDriver = Controllers.driverController.getTranslateXAxis() * MAX_ROBOT_TRANS_SPEED;
    double yDriver = Controllers.driverController.getTranslateYAxis() * MAX_ROBOT_TRANS_SPEED;
    double rotDriver = Controllers.driverController.getRotateAxis() * MAX_ROBOT_ROT_SPEED;
    ChassisSpeeds autoSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(autoTranslation.getX(), autoTranslation.getY(), rotScaledVel, currentPose.getRotation());
    ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDriver, yDriver, rotDriver, currentPose.getRotation());
    swerve.driveAllianceRelative(driverSpeeds.plus(autoSpeeds));
        

  }

  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
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

  public Pose2d getCoralObjective(Rotation2d controllerVectorAngle) {
    Pair<Pose2d, Double> bestReefPose = new Pair<Pose2d,Double>(null, 1000.0);
    ArrayList<Pose2d> reefPoses = new ArrayList<>(ReefPositions.getScoringPoses());
    // Logger.recordOutput("AlignToPose/reefPoses", Pose2d.struct, reefPoses.stream().toArray(Pose2d[]::new));
    for (int i = 0; i < reefPoses.size(); i++) {
      Pose2d reefPose = reefPoses.get(i);
      double dist = reefPose.getTranslation().getDistance(swerve.getPose().getTranslation());
      
      double vectorError = Math.abs(controllerVectorAngle.minus(reefPose.getTranslation().getAngle()).getRotations());

      double weight = dist + (vectorError * VECTOR_ERROR_SCALAR);
      if (weight < bestReefPose.getSecond()) {
        bestReefPose = new Pair<Pose2d, Double>(reefPose, weight);
      }
    }
    return bestReefPose.getFirst();
  }
}
