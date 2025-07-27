package frc.robot.commands;

import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ANGLE_THRESHOLD;
import static frc.robot.constants.MiscConstants.AUTO_ALIGN_DIST_THRESHOLD;
import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ENABLED;
import static frc.robot.constants.MiscConstants.BARGE_CENTER_DIST_X;
import static frc.robot.constants.MiscConstants.FIELD_CENTER_X;
import static frc.robot.constants.SwerveConstants.BARGE_ALIGN_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.DRIVE_CONTROLLER_PID;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructurePosition;
import frc.robot.subsystems.intake.GamePiece;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlignToPose;
import frc.robot.util.ArmvatorPosition;
import frc.robot.util.ObjectiveTracker;
import frc.robot.util.ReefPositions;

public class Teleop extends Command {
    Swerve swerve;
    Vision vision;
    Intake intake;
    Superstructure superStructure;
    ObjectiveTracker tracker;
    ProfiledPIDController bargeXController = new ProfiledPIDController(DRIVE_CONTROLLER_PID[0], DRIVE_CONTROLLER_PID[1], DRIVE_CONTROLLER_PID[2], BARGE_ALIGN_CONSTRAINTS);
    boolean isAligning = false;
    Debouncer bargeAlignPaddleDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public Teleop(Swerve swerve, Intake intake, ObjectiveTracker tracker, Superstructure superStructure) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.vision = swerve.vision;
        this.intake = intake;
        this.tracker = tracker;
        this.superStructure = superStructure;
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

    Pose2d currentCoralTarget = AlignToPose.getCoralObjective(swerve.getPose(), x, y);
    Logger.recordOutput("AlignToPose/AlignmentEnabled", AUTO_ALIGN_ENABLED);
    Logger.recordOutput("AlignToPose/2DAlignmentMode", VisionConstants.USE_2D_ALIGNMENT_MODE);

    boolean algaeAlignCommanded = bargeAlignPaddleDebouncer.calculate(Controllers.driverController.getRightPaddle().getAsBoolean());
    if (intake.getGamePiece() == GamePiece.ALGAE
        && (SuperstructurePosition.NET.equals(superStructure.desiredPosition)
            || ArmvatorPosition.NET.equals(superStructure.getClosestArmvatorPosition()))
        && algaeAlignCommanded) {
        
      
      if (isAligning == false) {
        // we just started the alignment process
        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(), swerve.getPose().getRotation());
        bargeXController.reset(swerve.getPose().getX(), currentSpeeds.vxMetersPerSecond);
      }
      isAligning = true;
      Logger.recordOutput("AlignToPose/TeleopAligning", "barge");

      double barge_x_coord;
      Rotation2d barge_align_angle;
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        barge_x_coord = FIELD_CENTER_X + BARGE_CENTER_DIST_X;
        barge_align_angle = Rotation2d.kZero;
      } else {
        barge_x_coord = FIELD_CENTER_X - BARGE_CENTER_DIST_X;
        barge_align_angle = Rotation2d.k180deg;
      }

      double bargeAlignSpeed = bargeXController.calculate(swerve.getPose().getX(), barge_x_coord);
      bargeAlignSpeed = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? (bargeAlignSpeed * -1) : bargeAlignSpeed;
      Logger.recordOutput("AlignToPose/bargeAlignSpeed", bargeAlignSpeed);

      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed + bargeAlignSpeed, ySpeed, rotSpeed),
          fieldAdjustedRobotRot), barge_align_angle);
    }
    // decide whether to do autoalign
    else if (currentCoralTarget.getTranslation()
            .getDistance(swerve.getPose().getTranslation()) < AUTO_ALIGN_DIST_THRESHOLD
            && Math.abs(currentCoralTarget.getRotation().minus(swerve.getPose().getRotation()).getRadians()) < AUTO_ALIGN_ANGLE_THRESHOLD
            && intake.getGamePiece() == GamePiece.CORAL
            && !tracker.facingReef()
            && AUTO_ALIGN_ENABLED) {
        int nearestTagId = ReefPositions.getClosestReefTagId(currentCoralTarget);
        Rotation2d gyroOffset = swerve.getPose().getRotation().minus(swerve.getPoseWheelsOnly().getRotation());
        isAligning = true;
        Logger.recordOutput("AlignToPose/TeleopAligning", "coral");
        // turn on 2d vision
        AlignToPose.enableReefVision(vision, gyroOffset, nearestTagId);

        // get final speeds
        Pair<ChassisSpeeds, Double> autoAlignSpeeds = AlignToPose.getAutoAlignSpeeds(x, y, swerve.getPose());
        ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed * (1 - autoAlignSpeeds.getSecond()), ySpeed * (1 - autoAlignSpeeds.getSecond()), rotSpeed * (1 - autoAlignSpeeds.getSecond())), fieldAdjustedRobotRot);
        swerve.drive(driverSpeeds.plus(autoAlignSpeeds.getFirst()));
    } else {
        isAligning = false;
        Logger.recordOutput("AlignToPose/TeleopAligning", "None");
        // return to normal vision
        AlignToPose.disableReefVision(vision);
        ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), fieldAdjustedRobotRot);
        swerve.drive(driverSpeeds);
    }

  }
}
