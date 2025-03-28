package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import java.util.Set;

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

  @AutoLogOutput
  GamePiece forcedGamePiece = null;
  
  public Intake() {
    configure();
    setDefaultCommand(holdGamePiece());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Color/Hue", canandcolor.getHSVHue());
    Logger.recordOutput("Intake/Color/Saturation", canandcolor.getHSVSaturation());
    Logger.recordOutput("Intake/Color/Value", canandcolor.getHSVValue());
    Logger.recordOutput("Intake/Color/Faults", canandcolor.getActiveFaults());
    Logger.recordOutput("Intake/Color/StickyFaults", canandcolor.getStickyFaults());
    Logger.recordOutput("Intake/Color/Connected", canandcolor.isConnected(1));
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

    canandcolor.clearStickyFaults();
  }

  public void setVoltage(double volts) {
    Logger.recordOutput("Intake/Volts", volts);
    intake.setVoltage(volts);
  }

  public Command runIntake() {
    return run(() -> setVoltage(INTAKE_VOLTAGE)).until(this::hasGamePiece).andThen(Commands.waitSeconds(0.1)).andThen(Commands.defer(() -> {
      if(getGamePiece() == GamePiece.ALGAE) {
        return Commands.waitSeconds(0.05);
      } else {
        return Commands.waitSeconds(0.5);
      }
    }, Set.of(this)));
  }

  @AutoLogOutput
  public double getPhosphorusProximity() {
    return canandcolor.getProximity();
  }

  @AutoLogOutput
  public boolean getPhosphorusAlgaeDIO() {
    return canandcolor.getDigoutState().getDigoutChannelValue(canandcolor.digout1().channelIndex());
  }

  public Command runOuttake(double volts) {
    return run(() -> setVoltage(-volts));
  }

  public Command outtakeCoral(double waitTime) {
    return run(() -> {setVoltage(CORAL_OUTTAKE_VOLTAGE);}).raceWith(
      Commands.sequence(
        Commands.waitUntil(() -> getGamePiece() == GamePiece.EMPTY),
        Commands.waitSeconds(waitTime)
      )
    );
  }

  public Command outtakeCoral() {
    return outtakeCoral(1.5);
  }

  public Command holdGamePiece() {
    return run(() -> {
      GamePiece currentGamePiece = getGamePiece();
      if (currentGamePiece == GamePiece.ALGAE) {
        setVoltage(INTAKE_STALL_VOLTAGE);
        return;
      }
      setVoltage(0);
    });
  }

  public Command stop() {
    return runOnce(() -> setVoltage(0));
  }

  @AutoLogOutput
  public GamePiece getGamePiecePhosphorus() {
  if (canandcolor.getProximity() < ALGAE_PROXIMITY_THRESHOLD && getPhosphorusAlgaeDIO()) {
    return GamePiece.ALGAE;
  } else if (canandcolor.getProximity() < CORAL_PROXIMITY_THRESHOLD) {
    return GamePiece.CORAL;
  }
    return GamePiece.EMPTY;
  }

  @AutoLogOutput
  public GamePiece getGamePiece() {
    if (forcedGamePiece != null) {
      return forcedGamePiece;
    }
    return getGamePiecePhosphorus();
  }

  public Command forceGamePiece(GamePiece piece) {
    return Commands.run(()->{
      forcedGamePiece = piece;
    }).finallyDo(()->{
      forcedGamePiece = null;
    });
  }

  public boolean hasGamePiece() {
    return getGamePiece() != GamePiece.EMPTY;
  }

  public boolean doesntHaveGamePiece() {
    return getGamePiece() == GamePiece.EMPTY;
  }

  
  public void setBrakeCoast(boolean willBrake) {
    intake.setCANTimeout(250);
    intakeConfig.idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    intake.configureAsync(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    intake.setCANTimeout(0);
    Logger.recordOutput("Wrist/isBraken", willBrake);
  }
}
