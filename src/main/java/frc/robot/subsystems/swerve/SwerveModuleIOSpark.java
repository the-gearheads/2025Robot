package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;

import java.util.Queue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private final Rotation2d offset;
    private final int modIndex;

    public SparkFlex drive;
    public SparkFlexConfig driveConfig = new SparkFlexConfig();
    public RelativeEncoder driveEncoder;
    public SparkClosedLoopController drivePid;

    public SparkMax steer;
    public SparkFlexConfig steerConfig = new SparkFlexConfig();
    public RelativeEncoder steerEncoder;
    public SparkClosedLoopController steerPid;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public SwerveModuleIOSpark(int modIndex, String moduleName) {
        this.modIndex = modIndex;
        offset = Rotation2d.fromDegrees(WHEEL_OFFSETS[modIndex]);
        drive = new SparkFlex(MOTOR_IDS[modIndex][0], MotorType.kBrushless);
        steer = new SparkMax(MOTOR_IDS[modIndex][1], MotorType.kBrushless);

        driveEncoder = drive.getEncoder();
        steerEncoder = steer.getAlternateEncoder();
        drivePid = drive.getClosedLoopController();
        steerPid = steer.getClosedLoopController();
        
        configureDrive();
        configureSteer();

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(steerEncoder::getPosition);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(offset))
                .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    }

    public void configureDrive() {
        drive.setCANTimeout(250);
        driveConfig.smartCurrentLimit(DRIVE_CURRENT_LIMIT);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.inverted(IS_INVERTED[modIndex]);
        driveConfig.voltageCompensation(12);
        driveConfig.encoder.positionConversionFactor(DRIVE_POS_FACTOR);
        driveConfig.encoder.velocityConversionFactor(DRIVE_VEL_FACTOR);

        driveConfig.closedLoop.p((DRIVE_PID[0] / 12.0) * DRIVE_VEL_FACTOR);
        driveConfig.closedLoop.i(0);
        driveConfig.closedLoop.d(0);

        driveConfig.signals.appliedOutputPeriodMs(20);
        driveConfig.signals.primaryEncoderPositionPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));
        driveConfig.signals.primaryEncoderVelocityPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));

        drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        drive.setCANTimeout(0);
    }

    public void configureSteer() {
        steer.setCANTimeout(250); 
        driveConfig.smartCurrentLimit(STEER_CURRENT_LIMIT);
        driveConfig.idleMode(IdleMode.kBrake);

        driveConfig.absoluteEncoder.positionConversionFactor(STEER_POS_FACTOR);
        driveConfig.absoluteEncoder.velocityConversionFactor(STEER_VEL_FACTOR);
        driveConfig.absoluteEncoder.inverted(true);

        driveConfig.closedLoop.positionWrappingEnabled(true);
        driveConfig.closedLoop.positionWrappingMinInput(0);
        driveConfig.closedLoop.positionWrappingMaxInput(Math.PI * 2);

        driveConfig.closedLoop.pidf(STEER_PIDF[0], STEER_PIDF[1], STEER_PIDF[2], STEER_PIDF[3]);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);


        // I currently do not know whether revlib takes the minumum of all signals in a status frame including or excluding defaults.
        driveConfig.signals.appliedOutputPeriodMs(20);
        driveConfig.signals.primaryEncoderPositionAlwaysOn(false);
        driveConfig.signals.primaryEncoderVelocityPeriodMs(40);

        driveConfig.signals.absoluteEncoderPositionAlwaysOn(true);
        driveConfig.signals.absoluteEncoderPositionPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));
        driveConfig.signals.absoluteEncoderVelocityAlwaysOn(true);
        driveConfig.signals.absoluteEncoderVelocityPeriodMs(20);

        steer.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        steer.setCANTimeout(0);
    }

    public void setDriveVolts(double volts) {
        drive.setVoltage(volts);
    }

    public void setSteerVolts(double volts) {
        steer.setVoltage(volts);
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public void resetSteerEncoder() {
        steerEncoder.setPosition(0);
    }

    

}
