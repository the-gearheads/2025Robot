package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {
    Rotation2d offset;
    
    String modulePath;

    public SwerveModule(int id, String moduleName) {
        this.modulePath = "Swerve/" + moduleName;
        this.offset = Rotation2d.fromDegrees(WHEEL_OFFSETS[id]);
        // if (Rob)
    }
}
