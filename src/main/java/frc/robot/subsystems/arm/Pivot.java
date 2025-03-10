package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ArmConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.vendor.ArmfeedforwardSettable;

public class Pivot extends SubsystemBase {
  /*
   * stand for parmensan arm
   */
  private SparkFlex pivot = new SparkFlex(PIVOT_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder pivotRelEnc;
  private SparkFlex pivotFollower = new SparkFlex(PIVOT_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  private SparkFlexConfig pivotConfig = new SparkFlexConfig();
  private SparkFlexConfig pivotFollowerConfig = new SparkFlexConfig();

  private Canandmag pivotAbsEnc = new Canandmag(PIVOT_ABS_ENCODER_ID);
  private ProfiledPIDController profiledPid = new ProfiledPIDController(PIVOT_PID[0], PIVOT_PID[1], PIVOT_PID[2],
      PIVOT_CONSTRAINTS,
      0.02);
  private ArmfeedforwardSettable pivotFeedforward = new ArmfeedforwardSettable(PIVOT_KS, PIVOT_KG, PIVOT_KV, PIVOT_KA);
  private PIDController pid = new PIDController(PIVOT_PID[0], PIVOT_PID[1], PIVOT_PID[2]);

  private RunMode defaultMode = RunMode.VOLTAGE;
  private RunMode mode = defaultMode;

  private ArmvatorSample sample;
  private double ff;
  private double output;
  private Double manualVoltage = 0.0;

  public Pivot() {
    configure();
    pivotRelEnc = pivot.getEncoder();
    profiledPid.reset(getAngle().getRadians());
    profiledPid.setGoal(getAngle().getRadians());
    pid.setTolerance(PIVOT_ANGLE_TOLERANCE);
    profiledPid.setTolerance(PIVOT_ANGLE_TOLERANCE);
  }

  @Override
  public void periodic() {

    if(DriverStation.isDisabled()) {
      pivotRelEnc.setPosition(getAbsAngle().getRadians());
    }

    switch (mode) {
      case PROFILED_PID:
        output = profiledPid.calculate(getAngle().getRadians()) + ff;

        // https://gist.github.com/person4268/46710dca9a128a0eb5fbd93029627a6b
        if (Math.abs(Units.radiansToDegrees(
            getAngle().getRadians() - profiledPid.getSetpoint().position)) > PIVOT_ANGLE_LIVE_FF_THRESHOLD) {
          ff = pivotFeedforward.calculate(getAngle().getRadians(), profiledPid.getSetpoint().velocity);
        } else {
          ff = pivotFeedforward.calculate(profiledPid.getSetpoint().position, profiledPid.getSetpoint().velocity);
        }

        output = profiledPid.calculate(getAngle().getRadians()) + ff;
        break;
      case VOLTAGE:
        output = manualVoltage;
        break;
      case TRAJECTORY:
        ff = pivotFeedforward.calculate(sample.armPos(), sample.armVel(), sample.armAccel());
        output = pid.calculate(getAngle().getRadians(), sample.armPos()) + ff;
        break;
    }

    if(mode != RunMode.PROFILED_PID) {
      profiledPid.reset(getAngleRad());
    } 

    if(mode != RunMode.TRAJECTORY) {
      pid.reset();
    }

    SmartDashboard.putData("Pivot/pidController", pid);
    Logger.recordOutput("Pivot/manualVoltage", manualVoltage);
    Logger.recordOutput("Pivot/Sample", sample);
    Logger.recordOutput("Pivot/attemptedOutput", output);
    // stops robot from runnign into itself
    if (output > 0 && getAbsAngle().getRadians() > MAX_ANGLE) {
      output = 0;
    }

    if (output < 0 && getAbsAngle().getRadians() < MIN_ANGLE) {
      output = 0;
    }

    if (mode == RunMode.PROFILED_PID) {
      if (profiledPid.getSetpoint().position < MIN_ANGLE || profiledPid.getSetpoint().position > MAX_ANGLE) {
        output = 0;
      }

      // Might as well just get as close as we can
      if (profiledPid.getGoal().position < MIN_ANGLE || profiledPid.getGoal().position > MAX_ANGLE) {
        profiledPid.setGoal(MathUtil.clamp(profiledPid.getGoal().position, MIN_ANGLE, MAX_ANGLE));
      }
    }

    Logger.recordOutput("Pivot/output", output);
    setMotorVoltage(output);
  }

  public void configure() {
    pivot.setCANTimeout(250);
    pivotFollower.setCANTimeout(250);

    pivotConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
    pivotConfig.voltageCompensation(12);
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(true);

    pivotConfig.signals.appliedOutputPeriodMs(10);
    pivotConfig.encoder.positionConversionFactor(PIVOT_POS_FACTOR);
    pivotConfig.encoder.velocityConversionFactor(PIVOT_VEL_FACTOR);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotFollowerConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
    pivotFollowerConfig.follow(pivot, true);
    pivotFollowerConfig.idleMode(IdleMode.kBrake);
    pivotFollowerConfig.voltageCompensation(12);

    pivotFollower.configure(pivotFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivot.setCANTimeout(0);
    pivotFollower.setCANTimeout(0);
  }

  @AutoLogOutput
  public Rotation2d getAbsAngle() {
    return new Rotation2d((PIVOT_ABS_ENCODER_OFFSET + Units.rotationsToRadians(pivotAbsEnc.getAbsPosition())) % (2 * Math.PI));
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return new Rotation2d(pivotRelEnc.getPosition());
  }

  public double getAngleRad() {
    return getAngle().getRadians();
  }

  @AutoLogOutput
  public double getVelocity() {
    return Units.rotationsToRadians(pivotAbsEnc.getVelocity());
  }

  @AutoLogOutput
  public Rotation2d getTargetAngle() {
    if (mode == RunMode.PROFILED_PID) {
      return new Rotation2d(profiledPid.getSetpoint().position);
    } else if (mode == RunMode.TRAJECTORY) {
      return new Rotation2d(pid.getSetpoint());
    } else {
      return getAngle();
    }
  }

  public void setGoalAngle(double angleRad) {
    angleRad = MathUtil.clamp(angleRad, MIN_ANGLE, MAX_ANGLE);
    profiledPid.setGoal(angleRad);
  }

  public void setSample(ArmvatorSample sample) {
    this.sample = sample;
  }

  public void setMode(RunMode mode) {
    this.mode = mode;
  }
  
  @AutoLogOutput
  public RunMode getMode() {
    return mode;
  }

  public void setVoltage(double volts) {
    manualVoltage = volts;
  }

  protected void setMotorVoltage(double volts) {
    pivot.setVoltage(output);
  }
  
  public void setVoltage(Voltage volts) {
    manualVoltage = volts.magnitude();
  }

  public boolean atPidSetpoint() {
    return mode == RunMode.TRAJECTORY && pid.atSetpoint();
  }

  public boolean atPoint(double angle) {
    return atPoint(angle, PIVOT_ANGLE_TOLERANCE);
  }

  public boolean atPoint(double angle, double tolerance) {
    return MathUtil.isNear(getAngle().getRadians(), angle, tolerance);
  }

  public SysIdRoutine getSysidRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.8).per(Seconds), Volts.of(3.2), null,
          (state) -> Logger.recordOutput("Pivot/SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism((Voltage v) -> {
        setMode(RunMode.VOLTAGE);
        setVoltage(v);
      }, null, this)
    );
  }

  public boolean forwardSysidLimit() {
    return getAngle().getRadians() > MAX_SYSID_ANGLE;
  }

  public boolean reverseSysidLimit() {
    return getAngle().getRadians() < MIN_SYSID_ANGLE;
  }

  public void setBrakeCoast(boolean willBrake) {
    pivot.setCANTimeout(250);
    pivotFollower.setCANTimeout(250);

    pivotConfig.idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotFollowerConfig.idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    pivot.configureAsync(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotFollower.configureAsync(pivotFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    pivot.setCANTimeout(0);
    pivotFollower.setCANTimeout(0);
    Logger.recordOutput("Pivot/isBraken", willBrake);
  }
}
