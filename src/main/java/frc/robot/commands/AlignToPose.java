package frc.robot.commands;

import static frc.robot.constants.MiscConstants.REEF_FACE_LENGTH;
import static frc.robot.constants.MiscConstants.MAX_REEF_LINEUP_DIST;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_DRIVE_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_ROT_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.XY_PATH_FOLLOWING_PID;
import static frc.robot.constants.SwerveConstants.ROT_PATH_FOLLOWING_PID;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.swerve.Swerve;

public class AlignToPose extends Command {
  Swerve swerve;
  Supplier<Pose2d> target;
  ProfiledPIDController driveController = new ProfiledPIDController(XY_PATH_FOLLOWING_PID[0], XY_PATH_FOLLOWING_PID[1],
      XY_PATH_FOLLOWING_PID[2], ALIGNMENT_DRIVE_CONSTRAINTS);
  ProfiledPIDController rotController = new ProfiledPIDController(ROT_PATH_FOLLOWING_PID[0], ROT_PATH_FOLLOWING_PID[1],
      ROT_PATH_FOLLOWING_PID[2], ALIGNMENT_ROT_CONSTRAINTS);

  public AlignToPose(Swerve swerve, Supplier<Pose2d> target) {
    this.swerve = swerve;
    this.target = target;
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerve.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(0,
            -new Translation2d(
                ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
                    swerve.getPose().getRotation()).vxMetersPerSecond,
                ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
                    swerve.getPose().getRotation()).vyMetersPerSecond)
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(swerve.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    double currentDistance = currentPose.relativeTo(target.get()).getTranslation().getNorm();

    double distanceScalar = MathUtil.clamp(-0.7679 * currentDistance + 1, 0, 1);
    driveController.calculate(currentDistance, 0.0);

    Pose2d currentTarget = getDriveTarget(currentPose, target.get());
    Logger.recordOutput("AlignToPose/DriveTarget", currentTarget);
    // u=(1−α)⋅udriver​+α⋅uauto​
    // scale α smoothly (quadratic?) based on distance from alignment pose
    // scale α based on velocity direction, if driver is going in similar direction
     // scale α based on controller direction, large changes will decrease α
    // g(θ)=0l5(1+cos(θ)) where theta is the different in driver direction and
    // commanded direction

    // all factors made 0 to 1 and then multiplied together, plug into original
    // equation.

    double targetDist = currentPose.getTranslation().getDistance(currentTarget.getTranslation());
    double driveVelocity = driveController.calculate(targetDist, 0.0);
    double driveScalar = driveVelocity * distanceScalar;
    double rotVelocity = rotController.calculate(currentPose.getRotation().getRadians(),
        currentTarget.getRotation().getRadians());
    Logger.recordOutput("AlignToPose/driveVelocitySetpoint", driveVelocity);
    Logger.recordOutput("AlignToPose/rotVelocitySetpoint", rotVelocity);

    var driveTranslation = new Pose2d(Translation2d.kZero,
        currentPose.getTranslation().minus(currentTarget.getTranslation()).getAngle())
        .transformBy(new Transform2d(new Translation2d(driveScalar, 0.0), Rotation2d.kZero))
        .getTranslation();
    driveTranslation.toVector().unit();
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveTranslation.getX(), driveTranslation.getY(), rotVelocity, currentPose.getRotation()));
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
    Logger.recordOutput("AlignToPose/offsetX", offset.getX());
    Logger.recordOutput("AlignToPose/offsetY", offset.getY());
    return goal.transformBy(
        new Transform2d(
            shiftXT * 1.5,
            Math.copySign(shiftYT * MAX_REEF_LINEUP_DIST * 0.8, offset.getY()), new Rotation2d()));
  }
}
