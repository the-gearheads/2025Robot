package frc.robot.commands;

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

        var speeds = new ChassisSpeeds(x, y, rot);

        swerve.driveFieldRelative(speeds);    
    }
}
