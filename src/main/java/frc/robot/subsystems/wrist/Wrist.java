package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.constants.WristConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructure.RunMode;

public class Wrist extends SubsystemBase {
  private SparkMax wrist = new SparkMax(WRIST_PIVOT_ID, MotorType.kBrushless);
  private SparkMaxConfig wristConfig = new SparkMaxConfig();

  private RelativeEncoder wristEncoder = wrist.getEncoder();
  private AbsoluteEncoder wristAbsEncoder = wrist.getAbsoluteEncoder();

  private ProfiledPIDController pid = new ProfiledPIDController(WRIST_PID[0], WRIST_PID[1], WRIST_PID[2],
      WRIST_CONSTRAINTS);

  double output;
  double ff;
  Double manualVoltage;

  public Wrist() {
    configure();
    wristEncoder.setPosition(wristAbsEncoder.getPosition());
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled())
      wristEncoder.setPosition(wristAbsEncoder.getPosition());

    ff = WRIST_FF.calculate(pid.getGoal().position, pid.getGoal().velocity);
    

    output = pid.calculate(getAngle().getRadians()) + ff;
    if (manualVoltage != null) {
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

    Logger.recordOutput("Wrist/output", output);
    setMotorVoltage(output);
  }

  public void configure() {
    wrist.setCANTimeout(250);
    wristConfig.smartCurrentLimit(WRIST_CURRENT_LIMIT);
    wristConfig.voltageCompensation(12);
    wristConfig.idleMode(IdleMode.kBrake);

    wristConfig.signals.appliedOutputPeriodMs(10);
    wristConfig.encoder.positionConversionFactor(WRIST_POS_FACTOR);
    wristConfig.encoder.velocityConversionFactor(WRIST_VEL_FACTOR);
    wristConfig.absoluteEncoder.velocityConversionFactor(WRIST_VEL_FACTOR);
    wristConfig.absoluteEncoder.positionConversionFactor(WRIST_POS_FACTOR);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    wrist.setCANTimeout(0);
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(wristAbsEncoder.getPosition());
  }

  @AutoLogOutput
  public double getVelocity() {
    return wristAbsEncoder.getVelocity();
  }

  public void setAngle(double angle) {
    pid.setGoal(angle);
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

  public boolean atPoint(double angle) {
    return MathUtil.isNear(getAngle().getRadians(), angle, WRIST_ANGLE_TOLERANCE);
  }

  public boolean atPoint(double angle, double tolerance) {
    return MathUtil.isNear(getAngle().getRadians(), angle, tolerance);
  }
}
