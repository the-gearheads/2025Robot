package frc.robot.commands;

import static frc.robot.constants.SwerveConstants.ALIGNMENT_DRIVE_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.ALIGNMENT_ROT_CONSTRAINTS;
import static frc.robot.constants.SwerveConstants.XY_PATH_FOLLOWING_PID;
import static frc.robot.constants.SwerveConstants.ROT_PATH_FOLLOWING_PID;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class AlignToPose extends Command {
  Swerve swerve;
  Supplier<Pose2d> target;
  ProfiledPIDController driveController = new ProfiledPIDController(XY_PATH_FOLLOWING_PID[0], XY_PATH_FOLLOWING_PID[1], XY_PATH_FOLLOWING_PID[2], ALIGNMENT_DRIVE_CONSTRAINTS);
  ProfiledPIDController rotController = new ProfiledPIDController(ROT_PATH_FOLLOWING_PID[0], ROT_PATH_FOLLOWING_PID[1], ROT_PATH_FOLLOWING_PID[2], ALIGNMENT_ROT_CONSTRAINTS);

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
    // Pose2d currentPose = swerve.getPose();
    // double currentDistance = currentPose.relativeTo(target.get()).getTranslation().getNorm();


    // u=(1−α)⋅udriver​+α⋅uauto​
    // scale α smoothly (quadratic?) based on distance from alignment pose
    // scale α based on velocity direction, if driver is going in similar direction to automatic adjustment, increase α
    // scale α based on controller direction, large changes will decrease α
    // g(θ)=0l5(1+cos(θ)) where theta is the different in driver direction and commanded direction

    // all factors made 0 to 1 and then multiplied together, plug into original equation.
  }


}
