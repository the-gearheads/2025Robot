package frc.robot.commands;

import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ANGLE_THRESHOLD;
import static frc.robot.constants.MiscConstants.AUTO_ALIGN_DIST_THRESHOLD;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.intake.GamePiece;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlignToPose;
import frc.robot.util.ObjectiveTracker;
import frc.robot.util.ReefPositions;

public class Teleop extends Command {
    Swerve swerve;
    // AlignToPose autoAlign = new AlignToPose();
    Vision vision;
    Intake intake;
    ObjectiveTracker tracker;

    public Teleop(Swerve swerve, Intake intake, ObjectiveTracker tracker) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.vision = swerve.vision;
        this.intake = intake;
        this.tracker = tracker;
    }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    Pose2d swervePose = swerve.getPose();
    Rotation2d swervePoseRotation = swervePose.getRotation();
    Rotation2d fieldAdjustedRobotRot = swervePoseRotation;
    ChassisSpeeds finalSpeeds;

    if (DriverStation.isAutonomous()) 
      swerve.drive(new ChassisSpeeds()); // shouldn't be needed but eh

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      fieldAdjustedRobotRot = fieldAdjustedRobotRot.rotateBy(Rotation2d.fromDegrees(180));
    }

    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    double xSpeed = Math.signum(x) * Math.pow(x, 2);
    double ySpeed = Math.signum(y) * Math.pow(y, 2);
    double rotSpeed = Math.signum(rot) * Math.pow(rot, 2);

    xSpeed *= MAX_ROBOT_TRANS_SPEED;
    ySpeed *= MAX_ROBOT_TRANS_SPEED;
    rotSpeed *= MAX_ROBOT_TRANS_SPEED;

    // decide whether to do autoalign
    Pose2d currentCoralTarget = AlignToPose.getCoralObjective(swervePose, x, y);
        
    if (currentCoralTarget.getTranslation()
          .getDistance(swervePose.getTranslation()) < AUTO_ALIGN_DIST_THRESHOLD
        && Math.abs(currentCoralTarget.getRotation().minus(swervePoseRotation).getRadians()) < AUTO_ALIGN_ANGLE_THRESHOLD
        && intake.getGamePiece() == GamePiece.CORAL
        && !tracker.facingReef()) {
      Logger.recordOutput("AlignToPose/TeleopAligning", true);
      setVisionTeleopAligning(ReefPositions.getClosestReefTagId(currentCoralTarget));
      finalSpeeds = getDriverSpeeds(fieldAdjustedRobotRot, x, y, rot, swervePose);
    } else {
      Logger.recordOutput("AlignToPose/TeleopAligning", false);
      setVisionTeleopNotAligning();
      finalSpeeds = getDriverSpeeds(fieldAdjustedRobotRot, xSpeed, ySpeed, rotSpeed);
    }
    swerve.drive(finalSpeeds);
  }

  private void setVisionTeleopAligning(int nearestTagId) {
    vision.setCameraPreference(1); // back right bc lower fov = probably better
    visionSetPoseStrategy(1);
    visionSetPoseStrategy(2);
    vision.filterTagById(1, nearestTagId);
    vision.filterTagById(2, nearestTagId);
    vision.disableCamera(0);
  }

  private void setVisionTeleopNotAligning() {
    vision.disableIdFiltering(1);
    vision.disableIdFiltering(2);
    vision.defaultPoseStrategies();
    vision.enableCamera(0);
  }

  private ChassisSpeeds getDriverSpeeds(Rotation2d fieldAdjustedRobotRot, double x, double y, double rot) {
    return calculateChassisDriveSpeed(fieldAdjustedRobotRot, x, y, rot);
  }

  private ChassisSpeeds getDriverSpeeds(Rotation2d fieldAdjustedRobotRot, double x, double y, double rot, Pose2d swervePose) {
    Pair<ChassisSpeeds, Double> autoAlignSpeeds = AlignToPose.getAutoAlignSpeeds(x, y, swervePose);
    double xSpeed = calculateChassisAxisSpeed(x, autoAlignSpeeds);
    double ySpeed = calculateChassisAxisSpeed(y, autoAlignSpeeds);
    double rotSpeed = calculateChassisAxisSpeed(rot, autoAlignSpeeds);
    return calculateChassisDriveSpeed(fieldAdjustedRobotRot, xSpeed, ySpeed, rotSpeed).plus(autoAlignSpeeds.getFirst());
  }

  private ChassisSpeeds calculateChassisDriveSpeed(Rotation2d fieldAdjustedRobotRot, double x, double y, double rot) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, rot), fieldAdjustedRobotRot);
  }

  private void visionSetPoseStrategy(int cameraIndex) {
    vision.setPoseStrategy(cameraIndex, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  private double calculateChassisAxisSpeed(double axis, Pair<ChassisSpeeds, Double> autoAlignSpeeds) {
    return axis * (1 - autoAlignSpeeds.getSecond());
  }
}
