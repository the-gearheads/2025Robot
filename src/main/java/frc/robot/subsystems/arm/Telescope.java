package frc.robot.subsystems.arm;

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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
  SparkFlex elevator = new SparkFlex(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  SparkFlex elevatorFollower = new SparkFlex(ELEVATOR_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  SparkFlexConfig elevatorConfig = new SparkFlexConfig();
  SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();
  RelativeEncoder elevatorEncoder = elevator.getEncoder();
  ProfiledPIDController elevatorPid = new ProfiledPIDController(ELEVATOR_PID[0], ELEVATOR_PID[1], ELEVATOR_PID[2],
      ELEVATOR_CONSTRAINTS);
    
  double ff;
  double output;
  double manualVoltage;

  public Telescope() {
    configure();
  }

  public void configure() {
    elevator.setCANTimeout(250);
    elevatorFollower.setCANTimeout(250);

    elevatorConfig.smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
    elevatorConfig.voltageCompensation(12);
    elevatorConfig.idleMode(IdleMode.kBrake);

    elevatorConfig.signals.appliedOutputPeriodMs(10);
    elevatorConfig.encoder.positionConversionFactor(ELEVATOR_POS_FACTOR);
    elevatorConfig.encoder.velocityConversionFactor(ELEVATOR_VEL_FACTOR);

    elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorFollowerConfig.smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
    elevatorFollowerConfig.follow(elevator);
    elevatorFollowerConfig.idleMode(IdleMode.kBrake);
    elevatorFollowerConfig.voltageCompensation(12);

    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    elevator.setCANTimeout(0);
    elevatorFollower.setCANTimeout(0);
  }

  @Override
  public void periodic() {
    ff = ELEVATOR_FEEDFORWARD.calculate(elevatorPid.getSetpoint().velocity);

    output = elevatorPid.calculate(getPosition()) + ff;
    Logger.recordOutput("Telescope/attemptedOutput", output);

    // stops robot from runnign into itself
    if (output > 0 && getPosition() > MAX_HEIGHT) {
      output = 0;
    }

    if (output < 0 && getPosition() < MIN_HEIGHT) {
      output = 0;
    }

    if (elevatorPid.getSetpoint().position < MIN_HEIGHT || elevatorPid.getSetpoint().position > MAX_HEIGHT) {
      output = 0;
    }

    // Might as well just get as close as we can
    if (elevatorPid.getGoal().position < MIN_HEIGHT || elevatorPid.getGoal().position > MAX_HEIGHT) {
      elevatorPid.setGoal(MathUtil.clamp(elevatorPid.getGoal().position, MIN_HEIGHT, MAX_HEIGHT));
    }
    
    Logger.recordOutput("Pivot/manualVoltage", manualVoltage);
    if (manualVoltage != 0) {
      output = manualVoltage;
      manualVoltage = 0;
    }
    Logger.recordOutput("Telescope/output", output);
    elevator.setVoltage(output);
  }

  @AutoLogOutput
  public boolean getLimitswitch() {
    return elevator.getReverseLimitSwitch().isPressed();
  }

  public void setPosition(double setpointLength) {
    elevatorPid.setGoal(setpointLength);
  }

  @AutoLogOutput
  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  @AutoLogOutput
  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  public void setVoltage(double volts) {
    manualVoltage = volts;
  }

  public void setVoltage(Voltage volts) {
    manualVoltage = volts.magnitude();
  }

  public void setEncoder(double position) {
    elevatorEncoder.setPosition(position);
  }

  public Command getHomingRoutine() {
    return this.startEnd(() -> {
      setVoltage(HOMING_VOLTAGE);
    }, () -> {
      setVoltage(0);
      setEncoder(0);
    }).until(this::getLimitswitch);
  }
}
