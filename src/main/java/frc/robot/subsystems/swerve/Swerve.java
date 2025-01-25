package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;

public class Swerve extends SubsystemBase {

    static final Lock odometryLock = new ReentrantLock();
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
    SwerveDrivePoseEstimator multitagPoseEstimator;
    Field2d field = new Field2d();
    Vision vision;

    int simGyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble simGyroAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simGyro, "Yaw"));
    PIDController headingController = new PIDController(5.2, 0, 0.5);

    SwerveModule[] modules = {
            new SwerveModule(0, "FL"),
            new SwerveModule(1, "FR"),
            new SwerveModule(2, "BL"),
            new SwerveModule(3, "BR")
    };

    public Swerve() {
        this.vision = new Vision(this);
        gyro.zeroYaw();
        SmartDashboard.putData("Field", field);

        for (SwerveModule module : modules) {
            module.configure();
        }

        SparkOdometryThread.getInstance().start();

        multitagPoseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(),
                new Pose2d());
        headingController.enableContinuousInput(0, 2 * Math.PI);
        headingController.setTolerance(HEADING_CONTROLLER_TOLERANCE);
    }

    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    @Override
    public void simulationPeriodic() {
        double degreesPerSecond = Units.radiansToDegrees(getRobotRelativeSpeeds().omegaRadiansPerSecond);
        simGyroAngle.set(simGyroAngle.get() - (degreesPerSecond * 0.02));
    }

    public void drive(ChassisSpeeds speeds, Double alignToAngle) {
        double commandedRot = headingController.calculate(getPose().getRotation().getRadians());

        if (alignToAngle != null) {
            Logger.recordOutput("Swerve/PoseRotPidAtSetpoint", headingController.atSetpoint());
            headingController.setSetpoint(alignToAngle);
            if (!headingController.atSetpoint()) {
                speeds.omegaRadiansPerSecond = commandedRot;
            } else {
                speeds.omegaRadiansPerSecond = 0;
            }
        }

        ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);

        Logger.recordOutput("Swerve/Speeds", speeds);
        Logger.recordOutput("Swerve/DiscretizedSpeeds", discretized);
        Logger.recordOutput("Swerve/DesiredStates", moduleStates);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(moduleStates[i]);
        }
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, null);
    }

    public void driveFieldRelative(ChassisSpeeds speeds, Double alignToAngle) {
        var rot = getPose().getRotation();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            rot = rot.rotateBy(Rotation2d.fromDegrees(180));
        }
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot), alignToAngle);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveFieldRelative(speeds, null);
    }

    public SwerveModulePosition[] getModulePositions() {
        getModuleStates();
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getCurrentModulePosition();
        }
        Logger.recordOutput("Swerve/Positions", positions);
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }
        Logger.recordOutput("Swerve/States", states);
        return states;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return multitagPoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        multitagPoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }
}
