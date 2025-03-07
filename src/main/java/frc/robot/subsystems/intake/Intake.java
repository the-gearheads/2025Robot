package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);
  RelativeEncoder intakeEncoder = intake.getEncoder();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  Debouncer stallDebouncer = new Debouncer(0.5);

  double manualVoltage;
  
  public Intake() {
    configure();
  }

  @Override
  public void periodic() {
    stallDebouncer.calculate(isCurrentlyStuck());
    Logger.recordOutput("Intake/outputVolts", manualVoltage);
    setMotorVoltage(manualVoltage);
  }

  @AutoLogOutput
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void configure() {
    intake.setCANTimeout(250);
    intakeConfig.smartCurrentLimit(INTAKE_CURRENT_LIMIT);
    intakeConfig.voltageCompensation(12);
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.encoder.velocityConversionFactor(INTAKE_VEL_FACTOR);
    intakeConfig.encoder.positionConversionFactor(INTAKE_POS_FACTOR);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intake.setCANTimeout(0);
  }

  public void setVoltage(double volts) {
    manualVoltage = volts;
  }

  protected void setMotorVoltage(double volts) {
    intake.setVoltage(volts);
  }

  public Command run() {
    return this.run(() -> {
      if(!stallDebouncer.calculate(isCurrentlyStuck())) {
        this.setVoltage(INTAKE_VOLTAGE);
      } else {
        this.setVoltage(INTAKE_STALL_VOLTAGE);
      }
    });
  }

  public Command stop() {
    return this.runOnce(() -> intake.setVoltage(0));
  }

  private boolean isCurrentlyStuck() {
    return (Math.abs(intakeEncoder.getVelocity()) < STALL_VELOCITY_THRESHOLD) && (Math.abs(manualVoltage) > 1);
  }
}
