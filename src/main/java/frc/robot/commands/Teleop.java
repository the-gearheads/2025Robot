package frc.robot.commands;

import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ANGLE_THRESHOLD;
import static frc.robot.constants.MiscConstants.AUTO_ALIGN_DIST_THRESHOLD;
import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ENABLED;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
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
    if(!DriverStation.isAutonomous()) {
      VisionConstants.USE_2D_ALIGNMENT_MODE = false;
    }
    if(DriverStation.isTeleop()) {
      vision.enable();
    }
  }

  @Override
  public void execute() {
    if(DriverStation.isAutonomous()) {
      swerve.drive(new ChassisSpeeds()); // shouldn't be needed but eh
      return;
    }
    var fieldAdjustedRobotRot = swerve.getPose().getRotation();
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

    ChassisSpeeds finalSpeeds;

    // decide whether to do autoalign
    Pose2d currentCoralTarget = AlignToPose.getCoralObjective(swerve.getPose(), x, y);
    Logger.recordOutput("AlignToPose/AlignmentEnabled", AUTO_ALIGN_ENABLED);
    Logger.recordOutput("AlignToPose/2DAlignmentMode", VisionConstants.USE_2D_ALIGNMENT_MODE);
    if (currentCoralTarget.getTranslation()
            .getDistance(swerve.getPose().getTranslation()) < AUTO_ALIGN_DIST_THRESHOLD
            && Math.abs(currentCoralTarget.getRotation().minus(swerve.getPose().getRotation()).getRadians()) < AUTO_ALIGN_ANGLE_THRESHOLD
            && intake.getGamePiece() == GamePiece.CORAL
            && !tracker.facingReef()
            && AUTO_ALIGN_ENABLED) {
        int nearestTagId = ReefPositions.getClosestReefTagId(currentCoralTarget);
        Rotation2d gyroOffset = swerve.getPose().getRotation().minus(swerve.getPoseWheelsOnly().getRotation());
        Logger.recordOutput("AlignToPose/TeleopAligning", true);
        // turn on 2d vision
        AlignToPose.enableReefVision(vision, gyroOffset, nearestTagId);

        // get final speeds
        Pair<ChassisSpeeds, Double> autoAlignSpeeds = AlignToPose.getAutoAlignSpeeds(x, y, swerve.getPose());
        ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x * (1 - autoAlignSpeeds.getSecond()), y * (1 - autoAlignSpeeds.getSecond()), rot * (1 - autoAlignSpeeds.getSecond())), fieldAdjustedRobotRot);
        finalSpeeds = driverSpeeds.plus(autoAlignSpeeds.getFirst());
    } else {
        Logger.recordOutput("AlignToPose/TeleopAligning", false);
        // return to normal vision
        AlignToPose.disableReefVision(vision);
        ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), fieldAdjustedRobotRot);
        finalSpeeds = driverSpeeds;
    }

    swerve.drive(finalSpeeds);
  }
}
