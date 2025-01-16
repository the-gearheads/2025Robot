package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private final Rotation2d offset;

    public SparkFlex drive;
    public SparkFlexConfig driveConfig = new SparkFlexConfig();
    public RelativeEncoder driveEncoder;
    public SparkClosedLoopController drivePid;

    public SparkMax steer;
    public SparkFlexConfig steerConfig = new SparkFlexConfig();
    public RelativeEncoder steerEncoder;
    public SparkClosedLoopController steerPid;
    public SwerveModuleIOSpark(int modIndex, String moduleName) {

        offset = Rotation2d.fromDegrees(WHEEL_OFFSETS[modIndex]);
        drive = new SparkFlex(MOTOR_IDS[modIndex][0], MotorType.kBrushless);
        steer = new SparkMax(MOTOR_IDS[modIndex][1], MotorType.kBrushless);
        driveEncoder = drive.getEncoder();
        steerEncoder = steer.getAlternateEncoder();
        drivePid = drive.getClosedLoopController();
        steerPid = steer.getClosedLoopController();

    }
}
