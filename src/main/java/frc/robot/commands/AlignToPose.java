package frc.robot.commands;

import static frc.robot.constants.MiscConstants.REEF_FACE_LENGTH;
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
import edu.wpi.first.wpilibj2.command.Command;
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

  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    double currentDistance = currentPose.relativeTo(target.get()).getTranslation().getNorm();

    double distanceScalar = MathUtil.clamp(-0.7679 * currentDistance + 1, 0, 1);
    driveController.calculate(currentDistance, 0.0);

    Logger.recordOutput("AlignToPose/DriveTarget", getDriveTarget(currentPose, target.get()));
    // u=(1−α)⋅udriver​+α⋅uauto​
    // scale α smoothly (quadratic?) based on distance from alignment pose
    // scale α based on velocity direction, if driver is going in similar direction
    // to automatic adjustment, increase α
    // scale α based on controller direction, large changes will decrease α
    // g(θ)=0l5(1+cos(θ)) where theta is the different in driver direction and
    // commanded direction

    // all factors made 0 to 1 and then multiplied together, plug into original
    // equation.
  }

  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    Pose2d offset = robot.relativeTo(goal);
    double xDist = Math.abs(offset.getX());
    double yDist = Math.abs(offset.getY());

    double shiftXT = MathUtil.clamp(
        (yDist / (REEF_FACE_LENGTH * 2)) + ((xDist - 0.3) / (REEF_FACE_LENGTH * 3)),
        0.0,
        1.0);
    double shiftYT = MathUtil.clamp(offset.getX() / REEF_FACE_LENGTH, 0.0, 1.0);

    return goal.transformBy(
        new Transform2d(
            shiftXT * 1.5,
            // Math.copySign(shiftYT * 1.5 * 0.8, offset.getY())
            0
            , new Rotation2d()
          )
        );
  }
}
