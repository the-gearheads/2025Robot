package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;

import java.util.Queue;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private final int modIndex;

    private final SparkFlex drive;
    private final SparkFlexConfig driveConfig = new SparkFlexConfig();
    private final RelativeEncoder driveEncoder;
    private final SparkClosedLoopController drivePid;
     
    private final SparkMax steer;
    private final SparkMaxConfig steerConfig = new SparkMaxConfig();
    private final SparkAbsoluteEncoder steerEncoder;
    private final SparkClosedLoopController steerPid;
    private final Rotation2d offset;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public SwerveModuleIOSpark(int modIndex, String moduleName) {
        this.modIndex = modIndex;
        drive = new SparkFlex(MOTOR_IDS[modIndex][0], MotorType.kBrushless);
        steer = new SparkMax(MOTOR_IDS[modIndex][1], MotorType.kBrushless);

        driveEncoder = drive.getEncoder();
        steerEncoder = steer.getAbsoluteEncoder();
        drivePid = drive.getClosedLoopController();
        steerPid = steer.getClosedLoopController();
        this.offset = new Rotation2d(WHEEL_OFFSETS[modIndex]);
        
        configureDrive();
        configureSteer();

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(this::getAngleRad);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(steerEncoder.getPosition()).minus(offset);
    }

    public double getAngleRad() {
        return getAngle().getRadians();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
    // odometry
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    
    // drive
    inputs.driveAppliedVolts = drive.getAppliedOutput() * drive.getBusVoltage();
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveTemperature = drive.getMotorTemperature();
    inputs.driveCurrentAmps = drive.getOutputCurrent();

    // steer
    inputs.steerAngle = getAngle();
    inputs.steerVelocityRadPerSec = steerEncoder.getVelocity();
    inputs.steerAppliedVolts = steer.getAppliedOutput() * steer.getBusVoltage();
    inputs.steerCurrentAmps = steer.getOutputCurrent();

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    }

    @Override
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

    @Override
    public void configureSteer() {
        steer.setCANTimeout(250); 
        steerConfig.smartCurrentLimit(STEER_CURRENT_LIMIT);
        steerConfig.idleMode(IdleMode.kBrake);

        steerConfig.absoluteEncoder.positionConversionFactor(STEER_POS_FACTOR);
        steerConfig.absoluteEncoder.velocityConversionFactor(STEER_VEL_FACTOR);
        steerConfig.absoluteEncoder.inverted(true);

        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.closedLoop.positionWrappingMinInput(0);
        steerConfig.closedLoop.positionWrappingMaxInput(Math.PI * 2);

        steerConfig.closedLoop.pidf(STEER_PIDF[0], STEER_PIDF[1], STEER_PIDF[2], STEER_PIDF[3]);
        steerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);


        // I currently do not know whether revlib takes the minumum of all signals in a status frame including or excluding defaults.
        steerConfig.signals.appliedOutputPeriodMs(20);
        steerConfig.signals.primaryEncoderPositionAlwaysOn(false);
        steerConfig.signals.primaryEncoderVelocityPeriodMs(40);

        steerConfig.signals.absoluteEncoderPositionAlwaysOn(true);
        steerConfig.signals.absoluteEncoderPositionPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));
        steerConfig.signals.absoluteEncoderVelocityAlwaysOn(true);
        steerConfig.signals.absoluteEncoderVelocityPeriodMs(20);

        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        steer.setCANTimeout(0);
    }

    @Override
    public void setDriveVolts(double volts) {
        drive.setVoltage(volts);
    }

    @Override
    public void setSteerVolts(double volts) {
        steer.setVoltage(volts);
    }

    @Override
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    @Override
    public void setDriveVelocity(double velRadPerSec) {
        drivePid.setReference(velRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, DRIVE_FEEDFORWARD.calculate(velRadPerSec));
    }

    @Override
    public void setAngle(Rotation2d angle) {
        steerPid.setReference(angle.plus(offset).getRadians(), ControlType.kPosition);
    }

}
