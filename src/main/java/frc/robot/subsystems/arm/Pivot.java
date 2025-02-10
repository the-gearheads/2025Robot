package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ArmConstants.*;

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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.RunMode;

public class Pivot extends SubsystemBase {
  /*
   * stand for parmensan arm
   */
  SparkFlex pivot = new SparkFlex(PIVOT_MOTOR_ID, MotorType.kBrushless);
  SparkFlex pivotFollower = new SparkFlex(PIVOT_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  SparkFlexConfig pivotConfig = new SparkFlexConfig();
  SparkFlexConfig pivotFollowerConfig = new SparkFlexConfig();

  RelativeEncoder pivotEncoder = pivot.getEncoder();
  DutyCycleEncoder pivotAbsEnc = new DutyCycleEncoder(0); // TODO: incorrect. need to zero properly.
  ProfiledPIDController pid = new ProfiledPIDController(PIVOT_PID[0], PIVOT_PID[1], PIVOT_PID[2], PIVOT_CONSTRAINTS,
      0.02);

  double ff;
  double output;
  double manualVoltage;

  RunMode defaultMode = RunMode.VOLTAGE;
  RunMode mode = defaultMode;

  public Pivot() {
    configure();
    pivotEncoder.setPosition(pivotAbsEnc.get());
    pid.reset(getAngle().getRadians());
    pid.setGoal(getAngle().getRadians());
  }

  public void periodic() {
    if (DriverStation.isDisabled())
      pivotEncoder.setPosition(pivotAbsEnc.get());

    switch (mode) {
      case PID:
        output = pid.calculate(getAngle().getRadians()) + ff;

        // https://gist.github.com/person4268/46710dca9a128a0eb5fbd93029627a6b
        if (Math.abs(Units.radiansToDegrees(getAngle().getRadians() - pid.getSetpoint().position)) > PIVOT_ANGLE_LIVE_FF_THRESHOLD) {
          ff = PIVOT_FEEDFORWARD.calculate(getAngle().getRadians(), pid.getSetpoint().velocity);
        } else {
          ff = PIVOT_FEEDFORWARD.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
        }

        output = pid.calculate(getAngle().getRadians()) + ff;
        break;
      case VOLTAGE:
        output = manualVoltage;
        break;
    }

    Logger.recordOutput("Pivot/Mode", mode.toString());
    Logger.recordOutput("Pivot/attemptedOutput", output);

    // stops robot from runnign into itself
    if (output > 0 && getAngle().getRadians() > MAX_ANGLE) {
      output = 0;
    }

    if (output < 0 && getAngle().getRadians() < MIN_ANGLE) {
      output = 0;
    }

    if (pid.getSetpoint().position < MIN_ANGLE || pid.getSetpoint().position > MAX_ANGLE) {
      output = 0;
    }

    // Might as well just get as close as we can
    if (pid.getGoal().position < MIN_ANGLE || pid.getGoal().position > MAX_ANGLE) {
      pid.setGoal(MathUtil.clamp(pid.getGoal().position, MIN_ANGLE, MAX_ANGLE));
    }
    
    Logger.recordOutput("Pivot/manualVoltage", manualVoltage);
    if (manualVoltage != 0) {
      output = manualVoltage;
      manualVoltage = 0;
    }
    Logger.recordOutput("Telescope/output", output);
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
    pid.setGoal(angleRad);
  }

  public void setManualVoltage(double volts) {
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

  public void setMode(RunMode mode) {
    this.mode = mode;
  }

  public Command runWithMode(Command cmd, RunMode tempMode) {
    return cmd
      .beforeStarting(() -> setMode(tempMode), this)
      .andThen(() -> setMode(defaultMode), this);
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.8).per(Seconds), Volts.of(6), null, 
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
        this::setVoltage,
        null,
        this
      )
    );
  }
}
