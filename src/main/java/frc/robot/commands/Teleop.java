package frc.robot.commands;

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
        double x = Controllers.driverController.getTranslateXAxis();
        double y = Controllers.driverController.getTranslateYAxis();
        double rot = Controllers.driverController.getRotateAxis();

        x = Math.signum(x) * Math.pow(x, 2);
        y = Math.signum(y) * Math.pow(y, 2);
        rot = Math.signum(rot) * Math.pow(rot, 2);

        x *= MAX_ROBOT_TRANS_SPEED;
        y *= MAX_ROBOT_TRANS_SPEED;
        rot *= MAX_ROBOT_TRANS_SPEED;

        var speeds = new ChassisSpeeds(x, y, rot);

        swerve.driveAllianceRelative(speeds);
    }
}
