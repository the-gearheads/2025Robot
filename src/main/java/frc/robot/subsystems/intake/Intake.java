package frc.robot.subsystems.Intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);
  SparkMaxConfig intakeConfig = new SparkMaxConfig();

  public Intake() {
    configure();
  }

  public void configure() {
    intake.setCANTimeout(250);
    intakeConfig.smartCurrentLimit(INTAKE_CURRENT_LIMIT);
    intakeConfig.voltageCompensation(12);
    intakeConfig.idleMode(IdleMode.kBrake);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intake.setCANTimeout(0);
  }
}
