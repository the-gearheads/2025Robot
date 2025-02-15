package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ArmConstants.MAX_ANGLE;
import static frc.robot.constants.ArmConstants.MIN_ANGLE;
import static frc.robot.constants.ArmConstants.PIVOT_ANGLE_LIVE_FF_THRESHOLD;
import static frc.robot.constants.ArmConstants.PIVOT_ANGLE_TOLERANCE;
import static frc.robot.constants.ArmConstants.PIVOT_CONSTRAINTS;
import static frc.robot.constants.ArmConstants.PIVOT_CURRENT_LIMIT;
import static frc.robot.constants.ArmConstants.PIVOT_FEEDFORWARD;
import static frc.robot.constants.ArmConstants.PIVOT_MOTOR_FOLLOWER_ID;
import static frc.robot.constants.ArmConstants.PIVOT_MOTOR_ID;
import static frc.robot.constants.ArmConstants.PIVOT_PID;
import static frc.robot.constants.ArmConstants.PIVOT_POS_FACTOR;
import static frc.robot.constants.ArmConstants.PIVOT_VEL_FACTOR;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.SuperStructure.RunMode;
import frc.robot.util.ArmvatorSample;

public class Pivot extends SubsystemBase {
  /*
   * stand for parmensan arm
   */
  SparkFlex pivot = new SparkFlex(PIVOT_MOTOR_ID, MotorType.kBrushless);
  SparkFlex pivotFollower = new SparkFlex(PIVOT_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  SparkFlexConfig pivotConfig = new SparkFlexConfig();
  SparkFlexConfig pivotFollowerConfig = new SparkFlexConfig();

  RelativeEncoder pivotEncoder = pivot.getEncoder();
  DutyCycleEncoder pivotAbsEnc = new DutyCycleEncoder(0);
  ProfiledPIDController profiliedPid = new ProfiledPIDController(PIVOT_PID[0], PIVOT_PID[1], PIVOT_PID[2],
      PIVOT_CONSTRAINTS,
      0.02);
  PIDController pid = new PIDController(PIVOT_PID[0], PIVOT_PID[1], PIVOT_PID[2]);

  RunMode defaultMode = RunMode.VOLTAGE;
  RunMode mode = defaultMode;

  private ArmvatorSample sample;
  private double ff;
  private double output;
  private Double manualVoltage;

  public Pivot() {
    configure();
    pivotEncoder.setPosition(pivotAbsEnc.get());
    profiliedPid.reset(getAngle().getRadians());
    profiliedPid.setGoal(getAngle().getRadians());
  }

  public void periodic() {
    if (DriverStation.isDisabled())
      pivotEncoder.setPosition(pivotAbsEnc.get());

    switch (mode) {
      case PID:
        output = profiliedPid.calculate(getAngle().getRadians()) + ff;

        // https://gist.github.com/person4268/46710dca9a128a0eb5fbd93029627a6b
        if (Math.abs(Units.radiansToDegrees(
            getAngle().getRadians() - profiliedPid.getSetpoint().position)) > PIVOT_ANGLE_LIVE_FF_THRESHOLD) {
          ff = PIVOT_FEEDFORWARD.calculate(getAngle().getRadians(), profiliedPid.getSetpoint().velocity);
        } else {
          ff = PIVOT_FEEDFORWARD.calculate(profiliedPid.getSetpoint().position, profiliedPid.getSetpoint().velocity);
        }

        output = profiliedPid.calculate(getAngle().getRadians()) + ff;
        break;
      case VOLTAGE:
        output = manualVoltage;
        break;
      case TRAJECTORY:
        ff = PIVOT_FEEDFORWARD.calculate(sample.armPos(), sample.armVel(), sample.armAccel());
        output = pid.calculate(getAngle().getRadians(), sample.armPos()) + ff;
        break;
    }

    Logger.recordOutput("Pivot/manualVoltage", manualVoltage);
    Logger.recordOutput("Pivot/Sample", sample);
    Logger.recordOutput("Pivot/attemptedOutput", output);
    // stops robot from runnign into itself
    if (output > 0 && getAngle().getRadians() > MAX_ANGLE) {
      output = 0;
    }

    if (output < 0 && getAngle().getRadians() < MIN_ANGLE) {
      output = 0;
    }

    if (profiliedPid.getSetpoint().position < MIN_ANGLE || profiliedPid.getSetpoint().position > MAX_ANGLE) {
      output = 0;
    }

    // Might as well just get as close as we can
    if (profiliedPid.getGoal().position < MIN_ANGLE || profiliedPid.getGoal().position > MAX_ANGLE) {
      profiliedPid.setGoal(MathUtil.clamp(profiliedPid.getGoal().position, MIN_ANGLE, MAX_ANGLE));
    }

    Logger.recordOutput("Pivot/output", output);
    pivot.setVoltage(output);
  }

  public void configure() {
    pivot.setCANTimeout(250);
    pivotFollower.setCANTimeout(250);

    pivotConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
    pivotConfig.voltageCompensation(12);
    pivotConfig.idleMode(IdleMode.kBrake);

    pivotConfig.signals.appliedOutputPeriodMs(10);
    pivotConfig.encoder.positionConversionFactor(PIVOT_POS_FACTOR);
    pivotConfig.encoder.velocityConversionFactor(PIVOT_VEL_FACTOR);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotFollowerConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
    pivotFollowerConfig.follow(pivot);
    pivotFollowerConfig.idleMode(IdleMode.kBrake);
    pivotFollowerConfig.voltageCompensation(12);

    pivotFollower.configure(pivotFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivot.setCANTimeout(0);
    pivotFollower.setCANTimeout(0);
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return new Rotation2d(pivotEncoder.getPosition());
  }

  @AutoLogOutput
  public double getVelocity() {
    return pivotEncoder.getVelocity();
  }

  public void setAngle(double angleRad) {
    angleRad = MathUtil.clamp(angleRad, MIN_ANGLE, MAX_ANGLE);
    profiliedPid.setGoal(angleRad);
  }

  public void setSample(ArmvatorSample sample) {
    this.sample = sample;
  }

  public void setMode(RunMode mode) {
    this.mode = mode;
  }
  
  public RunMode getMode() {
    return mode;
  }

  public void setVoltage(double volts) {
    manualVoltage = volts;
  }

  public void setVoltage(Voltage volts) {
    manualVoltage = volts.magnitude();
  }

  public boolean atPoint(double angle) {
    return MathUtil.isNear(getAngle().getRadians(), angle, PIVOT_ANGLE_TOLERANCE);
  }

  public boolean atPoint(double angle, double tolerance) {
    return MathUtil.isNear(getAngle().getRadians(), angle, tolerance);
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.8).per(Seconds), Volts.of(6), null,
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
            this::setVoltage,
            null,
            this));
  }
}
