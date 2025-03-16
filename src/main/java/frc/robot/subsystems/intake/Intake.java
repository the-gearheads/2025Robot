package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);
  RelativeEncoder intakeEncoder = intake.getEncoder();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  Canandcolor canandcolor = new Canandcolor(CANANDCOLOR_ID);

  GamePiece gamePiece = getGamePiece();

  double manualVoltage;
  
  public Intake() {
    configure();
    setDefaultCommand(this.run(()->{setVoltage(0);}));
  }

  @Override
  public void periodic() {
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

  public Command runIntake() {
    return this.run(() -> {
      GamePiece currentGamePiece = getGamePiece();
      if (currentGamePiece == GamePiece.CORAL) {
        setVoltage(0);
      } else if (currentGamePiece == GamePiece.ALGAE) {
        setVoltage(INTAKE_STALL_VOLTAGE);
      } else {
        setVoltage(INTAKE_VOLTAGE);
      }
    });
  }

  @AutoLogOutput
  public double getPhosphorusProximity() {
    return canandcolor.getProximity();
  }

  @AutoLogOutput
  public boolean getPhosphorusAlgaeDIO() {
    return canandcolor.getDigoutState().getDigoutChannelValue(canandcolor.digout1().channelIndex());
  }

  public Command runOuttake() {
    return this.run(() -> intake.setVoltage(-INTAKE_VOLTAGE));
  }

  public Command runOuttake(double volts) {
    return this.run(() -> intake.setVoltage(-volts));
  }

  public Command outtakeCoral() {
    return this.run(() -> {intake.setVoltage(CORAL_OUTTAKE_VOLTAGE);}).raceWith(
      Commands.sequence(
        Commands.waitUntil(() -> getGamePiece() == GamePiece.EMPTY),
        Commands.waitSeconds(1.5)
      )
    );
  }

  public Command stop() {
    return this.runOnce(() -> intake.setVoltage(0));
  }

  @AutoLogOutput
  public GamePiece getGamePiece() {
    if (canandcolor.getProximity() < CORAL_PROXIMITY_THRESHOLD) {
      return GamePiece.CORAL;
    }
    if (canandcolor.getProximity() < ALGAE_PROXIMITY_THRESHOLD && getPhosphorusAlgaeDIO()) {
      return GamePiece.ALGAE;
    }
    return GamePiece.EMPTY;
  }
}
