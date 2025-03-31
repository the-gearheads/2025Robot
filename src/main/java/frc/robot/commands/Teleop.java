package frc.robot.commands;

import static frc.robot.constants.SwerveConstants.DRIVE_FEEDFORWARD;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;
import static frc.robot.constants.SwerveConstants.WHEEL_POSITIONS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ImplicitModelFollower;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ObjectiveTracker;

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
    // Pose2d currentCoralTarget = AlignToPose.getCoralObjective(swerve.getPose(), x, y);
    
    // if (currentCoralTarget.getTranslation()
    //         .getDistance(swerve.getPose().getTranslation()) < AUTO_ALIGN_DIST_THRESHOLD
    //         && Math.abs(currentCoralTarget.getRotation().minus(swerve.getPose().getRotation()).getRadians()) < AUTO_ALIGN_ANGLE_THRESHOLD
    //         && intake.getGamePiece() == GamePiece.CORAL
    //         && !tracker.facingReef()) {
    //     int nearestTagId = ReefPositions.getClosestReefTagId(currentCoralTarget);
    //     Rotation2d gyroOffset = swerve.getPose().getRotation().minus(swerve.getPoseWheelsOnly().getRotation());
    //     Logger.recordOutput("AlignToPose/TeleopAligning", true);
    //     // turn on 2d vision
    //     AlignToPose.enableReefVision(vision, gyroOffset, nearestTagId);

    //     // get final speeds
    //     Pair<ChassisSpeeds, Double> autoAlignSpeeds = AlignToPose.getAutoAlignSpeeds(x, y, swerve.getPose());
    //     ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x * (1 - autoAlignSpeeds.getSecond()), y * (1 - autoAlignSpeeds.getSecond()), rot * (1 - autoAlignSpeeds.getSecond())), fieldAdjustedRobotRot);
    //     finalSpeeds = driverSpeeds.plus(autoAlignSpeeds.getFirst());
    // } else {
    //     Logger.recordOutput("AlignToPose/TeleopAligning", false);
    //     // return to normal vision
    //     AlignToPose.disableReefVision(vision);
    //     ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), fieldAdjustedRobotRot);
    //     finalSpeeds = driverSpeeds;
    // }
    finalSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    finalSpeeds = implicitModelFollowing(finalSpeeds); // man this is gonna mess with autoalign but here goesssss
    // swerve.drive(new ChassisSpeeds(1,1, 0));
    swerve.drive(finalSpeeds);
  }

  SimpleMotorFeedforward desiredFF = new SimpleMotorFeedforward(0, DRIVE_FEEDFORWARD.getKv(), DRIVE_FEEDFORWARD.getKa() * 0.5);
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  LinearSystem<N1, N1, N1> currentSystem = LinearSystemId.identifyVelocitySystem(DRIVE_FEEDFORWARD.getKv(), DRIVE_FEEDFORWARD.getKa());
  LinearSystem<N1, N1, N1> desiredSystem = LinearSystemId.identifyVelocitySystem(desiredFF.getKv(), desiredFF.getKa());

  @SuppressWarnings("unchecked")
  ImplicitModelFollower<N1, N1, N1>[] followers = new ImplicitModelFollower[] {
    new ImplicitModelFollower<>(currentSystem, desiredSystem),
    new ImplicitModelFollower<>(currentSystem, desiredSystem),
    new ImplicitModelFollower<>(currentSystem, desiredSystem),
    new ImplicitModelFollower<>(currentSystem, desiredSystem)
  };
  public ChassisSpeeds implicitModelFollowing(ChassisSpeeds desiredSpeeds) {
    var modStates = swerve.getModuleStates();

    var rot = swerve.getPose().getRotation();

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      rot = rot.rotateBy(Rotation2d.fromDegrees(180));
    }

    var desiredRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, rot);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(desiredRobotSpeeds);
    Logger.recordOutput("Teleop/DesiredStates", states);

    for (int i = 0; i < 4; i++) {
      states[i].speedMetersPerSecond = followers[i].calculate(VecBuilder.fill(modStates[i].speedMetersPerSecond), VecBuilder.fill(desiredFF.calculate(states[i].speedMetersPerSecond))).get(0, 0);
    }

    Logger.recordOutput("Teleop/AdjustedStates", states);
    return kinematics.toChassisSpeeds(states);
  }
}
