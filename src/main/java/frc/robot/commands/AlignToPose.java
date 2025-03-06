package frc.robot.commands;

import static frc.robot.constants.MiscConstants.REEF_FACE_LENGTH;
import static frc.robot.constants.MiscConstants.MAX_REEF_LINEUP_DIST;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_DRIVE_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_ROT_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_ROT_SPEED;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;
import static frc.robot.constants.SwerveConstants.XY_PATH_FOLLOWING_PID;
import static frc.robot.constants.SwerveConstants.ROT_PATH_FOLLOWING_PID;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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

public class AlignToPose extends Command {
  Swerve swerve;
  Supplier<Pose2d> target;
  ProfiledPIDController driveController = new ProfiledPIDController(XY_PATH_FOLLOWING_PID[0], XY_PATH_FOLLOWING_PID[1],
      XY_PATH_FOLLOWING_PID[2], ALIGNMENT_DRIVE_CONSTRAINTS);
  ProfiledPIDController rotController = new ProfiledPIDController(ROT_PATH_FOLLOWING_PID[0], ROT_PATH_FOLLOWING_PID[1],
      ROT_PATH_FOLLOWING_PID[2], ALIGNMENT_ROT_CONSTRAINTS);

  LinearFilter controllerXFilter = LinearFilter.highPass(0.1, 0.02);
  LinearFilter controllerYFilter = LinearFilter.highPass(0.1, 0.02);

  
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

    controllerXFilter.reset();
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    controllerXFilter.calculate(x);
    controllerYFilter.calculate(y);
    double distanceScalarSlope = -0.7679;
    SmartDashboard.putNumber("AlignToPose/DistanceScalarSlope", distanceScalarSlope);

  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    Pose2d currentTarget = target.get();
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    
    double controllerMagnitude = MathUtil.clamp((Math.abs(x) + Math.abs(y)) / 2, 0, 0.33);
    Rotation2d angleError = currentPose.getTranslation().minus(currentTarget.getTranslation()).getAngle();
    double targetDist = currentPose.getTranslation().getDistance(currentTarget.getTranslation());

    // calculate scaling factors
    // $$ y=8.6711x^{2}-5.89177x+1 $$
    // $$ \ce{H2O + H2O -> 2H2O}$$
    double controllerMagScalingFactor = 8.6711 * Math.pow((controllerMagnitude/2.0), 2) - 5.89177 * (controllerMagnitude/2.0) + 1;
    double driveScaledVel = driveController.calculate(targetDist, 0.0) * controllerMagScalingFactor;
    double rotScaledVel = rotController.calculate(angleError.getRadians(), 0.0) * controllerMagScalingFactor;
    var autoTranslation = new Pose2d(Translation2d.kZero,
        angleError)
        .transformBy(new Transform2d(new Translation2d(driveScaledVel, 0.0), Rotation2d.kZero))
        .getTranslation();

    double xDriver = Controllers.driverController.getTranslateXAxis() * MAX_ROBOT_TRANS_SPEED;
    double yDriver = Controllers.driverController.getTranslateYAxis() * MAX_ROBOT_TRANS_SPEED;
    double rotDriver = Controllers.driverController.getRotateAxis() * MAX_ROBOT_ROT_SPEED;
    
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(autoTranslation.getX() + xDriver * (1-controllerMagScalingFactor), autoTranslation.getY() + yDriver * (1-controllerMagScalingFactor), rotScaledVel + rotDriver * (1-controllerMagScalingFactor), currentPose.getRotation()));

  }

//   @Override
//   public void execute() {
//     Pose2d currentPose = swerve.getPose();
//     double x = Controllers.driverController.getTranslateXAxis();
//     double y = Controllers.driverController.getTranslateYAxis();
//     Translation2d controllerTranslation = new Translation2d(x, y);
//     double currentDistance = currentPose.relativeTo(target.get()).getTranslation().getNorm();
//     // double highPassValue = (Math.abs(controllerXFilter.calculate(x)) + Math.abs(controllerYFilter.calculate(y))) / 2.0;
//     // Logger.recordOutput("AlignToPose/highpassFilterOut", highPassValue);
//     // 0.4
    
//     double distanceScalarSlope = SmartDashboard.getNumber("AlignToPose/DistanceScalarSlope", 0.5);

//     double distanceScalar = MathUtil.clamp(distanceScalarSlope * currentDistance + 1, 0, 1);
//     driveController.calculate(currentDistance, 0.0);

//     Pose2d currentTarget = getDriveTarget(currentPose, target.get());
//     Logger.recordOutput("AlignToPose/DriveTarget", currentTarget);
//     // u=(1−α)⋅udriver​+α⋅uauto​
//     // scale α smoothly (quadratic?) based on distance from alignment pose
//     // scale α based on velocity direction, if driver is going in similar direction
//      // scale α based on controller direction, large changes will decrease α
     
//     // all factors made 0 to 1 and then multiplied together, plug into original
//     // equation.

    // double targetDist = currentPose.getTranslation().getDistance(currentTarget.getTranslation());
//     double driveVelocity = driveController.calculate(targetDist, 0.0);
    // double rotVelocity = rotController.calculate(currentPose.getRotation().getRadians(),
//     currentTarget.getRotation().getRadians());
//     Logger.recordOutput("AlignToPose/driveVelocitySetpoint", driveVelocity);
//     Logger.recordOutput("AlignToPose/rotVelocitySetpoint", rotVelocity);
    
    // Rotation2d angleError = currentPose.getTranslation().minus(currentTarget.getTranslation()).getAngle();
//     double translationVectorError = angleError.getRadians() - controllerTranslation.getAngle().getRadians();
//     double vectorErrorScalar = MathUtil.clamp(-1.53 * translationVectorError + 0.8, 0, 1);
//     Logger.recordOutput("AlignToPose/vectorErrorScalar", vectorErrorScalar);
//     Logger.recordOutput("AlignToPose/translationVectorError", translationVectorError);
    
//     boolean vectorErrorThreshold = Math.abs(x) + Math.abs(y) > 0.4; 
//     if (vectorErrorThreshold)
//         vectorErrorScalar = 1;
//     double totalScalingFactor = distanceScalar * vectorErrorScalar;
    // double driveScaledVel = driveVelocity * totalScalingFactor;
    // var autoTranslation = new Pose2d(Translation2d.kZero,
    //     angleError)
    //     .transformBy(new Transform2d(new Translation2d(driveScaledVel, 0.0), Rotation2d.kZero))
    //     .getTranslation();


//     // driver controls
    // double xDriver = Controllers.driverController.getTranslateXAxis() * MAX_ROBOT_TRANS_SPEED;
    // double yDriver = Controllers.driverController.getTranslateYAxis() * MAX_ROBOT_TRANS_SPEED;
    // double rotDriver = Controllers.driverController.getRotateAxis() * MAX_ROBOT_ROT_SPEED;
    
    // swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(autoTranslation.getX() + xDriver * (1-totalScalingFactor), autoTranslation.getY() + yDriver * (1-totalScalingFactor), rotVelocity * distanceScalar + rotDriver * (1-totalScalingFactor), currentPose.getRotation()));
//   }

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
