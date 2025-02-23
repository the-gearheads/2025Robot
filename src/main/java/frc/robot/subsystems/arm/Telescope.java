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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SuperStructure.RunMode;
import frc.robot.util.ArmvatorSample;

public class Telescope extends SubsystemBase {
  SparkFlex elevator = new SparkFlex(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  SparkFlex elevatorFollower = new SparkFlex(ELEVATOR_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  SparkFlexConfig elevatorConfig = new SparkFlexConfig();
  SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();
  RelativeEncoder elevatorEncoder = elevator.getEncoder();
  ProfiledPIDController profiliedPid = new ProfiledPIDController(ELEVATOR_PID[0], ELEVATOR_PID[1], ELEVATOR_PID[2],
      ELEVATOR_CONSTRAINTS);
  PIDController pid = new PIDController(ELEVATOR_PID[0], ELEVATOR_PID[1], ELEVATOR_PID[2]);

  RunMode defaultMode = RunMode.VOLTAGE;
  RunMode mode = defaultMode;
  ArmvatorSample sample;
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
    switch (mode) {
      case PROFILED_PID:
        ff = ELEVATOR_FEEDFORWARD.calculate(profiliedPid.getSetpoint().velocity);
        output = profiliedPid.calculate(getPosition() - MIN_ABSOLUTE_HEIGHT) + ff;
        break;
      case PID:
        ff = ELEVATOR_FEEDFORWARD.calculate(sample.elevatorVel(), sample.elevatorAccel());
        output = pid.calculate(getPosition(), sample.elevatorLen()-MIN_ABSOLUTE_HEIGHT);
        break;
      case VOLTAGE:
        output = manualVoltage;
        break;
    }

    SmartDashboard.putData(pid);
    Logger.recordOutput("Telescope/pidSetpoint", pid.getSetpoint());
    Logger.recordOutput("Telescope/profiliedPIDSetpoint", profiliedPid.getSetpoint().position);
    Logger.recordOutput("Telescope/attemptedOutput", output);
    Logger.recordOutput("Telescope/manualVoltage", manualVoltage);
    Logger.recordOutput("Telescope/sample", sample);

    // stops robot from runnign into itself
    // if (output > 0 && getPosition() > MAX_HEIGHT) {
    //   output = 0;
    // }

    // if (output < 0 && getPosition() < MIN_HEIGHT) {
    //   output = 0;
    // }

    if (mode == RunMode.PROFILED_PID) {
      if (profiliedPid.getSetpoint().position < MIN_RELATIVE_HEIGHT || profiliedPid.getSetpoint().position > MAX_HEIGHT) {
        output = 0;
        // Might as well just get as close as we can
        if (profiliedPid.getGoal().position < MIN_RELATIVE_HEIGHT || profiliedPid.getGoal().position > MAX_HEIGHT) {
          profiliedPid.setGoal(MathUtil.clamp(profiliedPid.getGoal().position, MIN_RELATIVE_HEIGHT, MAX_HEIGHT));
        }
      }
    }


    Logger.recordOutput("Telescope/output", output);
    setMotorVoltage(output);
  }

  @AutoLogOutput
  public boolean getLimitswitch() {
    return elevator.getReverseLimitSwitch().isPressed();
  }

  public void setPosition(double setpointLength) {
    profiliedPid.setGoal(setpointLength);
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

  @AutoLogOutput
  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  @AutoLogOutput
  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  protected void setMotorVoltage(double voltage) {
    elevator.setVoltage(output);
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

  public SysIdRoutine getSysidRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.25).per(Seconds), Volts.of(0.2), null,
          (state) -> Logger.recordOutput("Telescope/SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism((Voltage v) -> {
        setMode(RunMode.VOLTAGE);
        setVoltage(v);
      }, null, this)
    );
  }

  public boolean getSysidForwardLimit() {
    return getPosition() > MAX_SYSID_HEIGHT;
  }

  public boolean getSysidReverseLimit() {
    return getPosition() < MIN_SYSID_HEIGHT;
  }
}
