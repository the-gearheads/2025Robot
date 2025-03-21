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
    AlignToPose autoAlign = new AlignToPose();
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
    if(DriverStation.isAutonomous()) swerve.drive(new ChassisSpeeds()); // shouldn't be needed but eh
    var fieldAdjustedRobotRot = swerve.getPose().getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      fieldAdjustedRobotRot = fieldAdjustedRobotRot.rotateBy(Rotation2d.fromDegrees(180));
    }

    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    double xSpeed = calculateSpeed(x);
    double ySpeed = calculateSpeed(y);
    double rotSpeed = calculateSpeed(rot);

    ChassisSpeeds finalSpeeds;
    // decide whether to do autoalign
    Pose2d currentCoralTarget = autoAlign.getCoralObjective(swerve.getPose(), x, y);
        
        if (currentCoralTarget.getTranslation()
                .getDistance(swerve.getPose().getTranslation()) < AUTO_ALIGN_DIST_THRESHOLD
                && Math.abs(currentCoralTarget.getRotation().minus(swerve.getPose().getRotation()).getRadians()) < AUTO_ALIGN_ANGLE_THRESHOLD
                && intake.getGamePiece() == GamePiece.CORAL
                && !tracker.facingReef()) {
            int nearestTagId = ReefPositions.getClosestReefTagId(currentCoralTarget);
            Logger.recordOutput("AlignToPose/TeleopAligning", true);
            vision.setCameraPreference(1); // back right bc lower fov = probably better
            vision.setPoseStrategy(1, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            vision.setPoseStrategy(2, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            vision.filterTagById(1, nearestTagId);
            vision.filterTagById(2, nearestTagId);
            vision.disableCamera(0);
            Pair<ChassisSpeeds, Double> autoAlignSpeeds = autoAlign.getAutoAlignSpeeds(x, y, swerve.getPose());
            ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x * (1 - autoAlignSpeeds.getSecond()), y * (1 - autoAlignSpeeds.getSecond()), rot * (1 - autoAlignSpeeds.getSecond())), fieldAdjustedRobotRot);
            finalSpeeds = driverSpeeds.plus(autoAlignSpeeds.getFirst());
            // finalSpeeds = driverSpeeds;
        } else {
            Logger.recordOutput("AlignToPose/TeleopAligning", false);
            vision.disableIdFiltering(1);
            vision.disableIdFiltering(2);
            vision.defaultPoseStrategies();
            vision.enableCamera(0);
            ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), fieldAdjustedRobotRot);
            finalSpeeds = driverSpeeds;
        }

    swerve.drive(finalSpeeds);
  }

  private double calculateSpeed(double val) {
    return Math.signum(val) * Math.pow(val, 2)*MAX_ROBOT_TRANS_SPEED;
  }
}
