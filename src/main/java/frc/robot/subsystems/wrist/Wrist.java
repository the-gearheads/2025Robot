package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.WristConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Wrist extends SubsystemBase {
  private SparkMax wrist = new SparkMax(WRIST_PIVOT_ID, MotorType.kBrushless);
  private SparkMaxConfig wristConfig = new SparkMaxConfig();

  private RelativeEncoder wristEncoder = wrist.getEncoder();
  private AbsoluteEncoder wristAbsEncoder = wrist.getAbsoluteEncoder();

  private Debouncer stallDebouncer = new Debouncer(0.5);
  private boolean isCurrentlyStalled = false;
  protected ProfiledPIDController pid = new ProfiledPIDController(WRIST_PID[0], WRIST_PID[1], WRIST_PID[2],
      WRIST_CONSTRAINTS);

  double output;
  double ff;
  Double manualVoltage;

  public Wrist() {
    configure();
    wristEncoder.setPosition(wristAbsEncoder.getPosition());
    pid.setGoal(getAngle().getRadians());
    pid.reset(getAngle().getRadians(), 0);
    // pid.setGoal(getAngle().getRadians());
    // pid.reset(getAngle().getRadians(), 0);
    
  }

  @AutoLogOutput
  int enabledSyncCount = 0;

  @Override
  public void periodic() {
    isCurrentlyStalled = stallDebouncer.calculate(isCurrentlyStuck());
    Logger.recordOutput("Wrist/isStuck", isCurrentlyStuck());
    Logger.recordOutput("Wrist/isStalled", isCurrentlyStalled);
    if (DriverStation.isDisabled()) {
      syncIntegratedEncoder();
      pid.setGoal(getAngle().getRadians()); // TODO: controversial?
    }

    if (DriverStation.isEnabled() && Math.abs(getAngle().minus(getIntregratedEncoderAngle()).getDegrees()) > 5) {
      syncIntegratedEncoder();
      enabledSyncCount++;
    }

    double ff = WRIST_FF.calculate(pid.getSetpoint().position + WRIST_FF_OFFSET_RAD, pid.getSetpoint().velocity);

    double pidVolts = pid.calculate(getIntregratedEncoderAngle().getRadians());
    double output = ff + pidVolts;

    Logger.recordOutput("Wrist/ffVolts", ff);
    Logger.recordOutput("Wrist/pidVolts", pidVolts);

    // output = 0;
    if (manualVoltage != null) {
      Logger.recordOutput("Wrist/manualVoltage", manualVoltage);
      output = manualVoltage;
      manualVoltage = null;
    }
    Logger.recordOutput("Wrist/attemptedOutput", output);
    if (output > 0 && getAngle().getRadians() > MAX_WRIST_ANGLE) {
      output = 0;
    }

    if (output < 0 && getAngle().getRadians() < MIN_WRIST_ANGLE) {
      output = 0;
    }

    if (pid.getGoal().position < MIN_WRIST_ANGLE || pid.getGoal().position > MAX_WRIST_ANGLE) {
      pid.setGoal(MathUtil.clamp(pid.getGoal().position, MIN_WRIST_ANGLE, MAX_WRIST_ANGLE));
    }

    if (isCurrentlyStalled) {
      output = 0;
      System.out.println("AAAAHH THE WRIST IS STALLED");
    }

    Logger.recordOutput("Wrist/output", output);
    setMotorVoltage(output);
  }

  public void configure() {
    wrist.setCANTimeout(250);
    wristConfig.smartCurrentLimit(WRIST_CURRENT_LIMIT);
    wristConfig.voltageCompensation(12);
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.inverted(true);

    wristConfig.signals.appliedOutputPeriodMs(10);
    wristConfig.encoder.positionConversionFactor(WRIST_POS_FACTOR);
    wristConfig.encoder.velocityConversionFactor(WRIST_VEL_FACTOR);
    wristConfig.absoluteEncoder.velocityConversionFactor(WRIST_ABS_VEL_FACTOR);
    wristConfig.absoluteEncoder.positionConversionFactor(WRIST_ABS_POS_FACTOR);
    wristConfig.absoluteEncoder.zeroCentered(true);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    wrist.setCANTimeout(0);
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(wristAbsEncoder.getPosition());
  }

  @AutoLogOutput
  public Rotation2d getIntregratedEncoderAngle() {
    return Rotation2d.fromRadians(wristEncoder.getPosition());
  }

  @AutoLogOutput
  public Rotation2d getTargetAngle() {
    return Rotation2d.fromRadians(pid.getGoal().position);
  }

  @AutoLogOutput
  public double getVelocity() {
    return wristAbsEncoder.getVelocity();
  }

  public void syncIntegratedEncoder() {
    wristEncoder.setPosition(wristAbsEncoder.getPosition());
  }

  public void setGoal(Rotation2d angle) {
    pid.setGoal(angle.getRadians());
  }

  public void resetProfiledPidTo(double angle) {
    pid.reset(angle, 0);
  }

  public void setGoal(double angle) {
    pid.setGoal(angle);
  }

  @AutoLogOutput
  public Rotation2d getGoalAngle() {
    return Rotation2d.fromRadians(pid.getGoal().position);
  }

  public void setVoltage(double volts) {
    manualVoltage = volts;
  }

  public void setVoltage(Voltage volts) {
    manualVoltage = volts.magnitude();
  }

  protected void setMotorVoltage(double volts) {
    wrist.setVoltage(volts);
  }

  public boolean atPoint(Rotation2d angle) {
    return atPoint(angle, WRIST_ANGLE_TOLERANCE);
  }

  public boolean atPoint(Rotation2d angle, double tolerance) {
    return MathUtil.isNear(getAngle().getRadians(), angle.getRadians(), tolerance);
  }

  @AutoLogOutput
  public boolean atPidSetpoint() {
    pid.setTolerance(WRIST_ANGLE_TOLERANCE);
    return pid.atSetpoint();
  }

  public boolean forwardSysidLimit() {
    return getAngle().getRadians() > MAX_SYSID_ANGLE;
  }

  public boolean reverseSysidLimit() {
    return getAngle().getRadians() < MIN_SYSID_ANGLE;
  }

  public SysIdRoutine getSysidRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(.5).per(Second), Volts.of(2.5), null,
          (state) -> Logger.recordOutput("Wrist/SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(this::setVoltage, null, this)
    );
  }

  public void setBrakeCoast(boolean willBrake) {
    wrist.setCANTimeout(250);
    wristConfig.idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    wrist.configureAsync(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    wrist.setCANTimeout(0);
    Logger.recordOutput("Wrist/isBraken", willBrake);
  }

  public Command goTo(Rotation2d angle, double tolerance) {
    return this.run(() -> {
      setGoal(angle);
    }).until(() -> {return this.atPoint(angle, tolerance);});
  }

  private boolean isCurrentlyStuck() {
    return (Math.abs(getVelocity()) < WRIST_STALL_VELOCITY_THRESHOLD) && manualVoltage > 1;
  }
}
