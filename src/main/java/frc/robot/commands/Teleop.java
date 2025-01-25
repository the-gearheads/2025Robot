package frc.robot.commands;

import static frc.robot.constants.SwerveConstants.MAX_ROBOT_ROT_SPEED;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;

public class Teleop extends Command {
    Swerve swerve;
    
    public Teleop(Swerve swerve) {
        addRequirements(swerve);
        this.swerve = swerve;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = Controllers.driverController.getTranslateXAxis() * MAX_ROBOT_TRANS_SPEED;
        double y = Controllers.driverController.getTranslateYAxis() * MAX_ROBOT_TRANS_SPEED;
        double rot = Controllers.driverController.getRotateAxis() * MAX_ROBOT_ROT_SPEED;

        var speeds = new ChassisSpeeds(x, y, rot);

        swerve.driveFieldRelative(speeds);
    }
}
