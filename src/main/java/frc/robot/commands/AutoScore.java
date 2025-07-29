package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class AutoScore extends Command {
    Swerve swerve;
    Vision vision;
    Intake intake;
    Superstructure superStructure;

    public AutoScore(Swerve swerve, Vision vision, Intake intake, Superstructure superStructure) {
        this.swerve = swerve;
        this.vision = vision;
        this.intake = intake;
        this.superStructure = superStructure;
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
    }
}
