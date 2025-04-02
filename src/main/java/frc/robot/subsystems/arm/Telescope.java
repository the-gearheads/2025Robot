package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.vendor.ElevatorFeedforwardSettable;
import frc.robot.util.vendor.PIDControllerCustomPeriod;

public class Telescope extends SubsystemBase {
  private SparkFlex elevator = new SparkFlex(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex elevatorFollower = new SparkFlex(ELEVATOR_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  private SparkFlexConfig elevatorConfig = new SparkFlexConfig();
  private SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();
  private RelativeEncoder elevatorEncoder = elevator.getEncoder();
  

  private ElevatorFeedforwardSettable elevatorFeedforward = new ElevatorFeedforwardSettable(ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

  private ProfiledPIDController profiledPid = new ProfiledPIDController(ELEVATOR_PID[0], ELEVATOR_PID[1], ELEVATOR_PID[2],
      ELEVATOR_CONSTRAINTS);
  private PIDControllerCustomPeriod pid = new PIDControllerCustomPeriod(ELEVATOR_PID[0], ELEVATOR_PID[1], ELEVATOR_PID[2]);

  private RunMode defaultMode = RunMode.PROFILED_PID;
  private RunMode mode = defaultMode;
  private ArmvatorSample sample;

  private double manualVoltage;
  private boolean isHomed = false;

  private double lastTimestamp = Timer.getFPGATimestamp();

  protected DoubleSupplier pivotAngleRadSupplier = () -> {return Math.PI / 2;};

  public Telescope() {
    configure();
    profiledPid.setTolerance(ELEVATOR_LENGTH_TOLERANCE);
    profiledPid.reset(getExtension(), getVelocity());
    pid.setTolerance(ELEVATOR_LENGTH_TRAJ_START_TOLERANCE);
  }

  public void setPivotAngleRadSupplier(DoubleSupplier pivotAngleRadSupplier) {
    this.pivotAngleRadSupplier = pivotAngleRadSupplier;
  }

  public void configure() {
    elevator.setCANTimeout(250);
    elevatorFollower.setCANTimeout(250);

    elevatorConfig.smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
    // elevatorConfig.voltageCompensation(12); // TODO: ?
    elevatorConfig.idleMode(IdleMode.kBrake);

    elevatorConfig.signals.appliedOutputPeriodMs(10);
    elevatorConfig.encoder.positionConversionFactor(ELEVATOR_POS_FACTOR);
    elevatorConfig.encoder.velocityConversionFactor(ELEVATOR_VEL_FACTOR);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
    elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorFollowerConfig.smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
    elevatorFollowerConfig.follow(elevator, true);
    elevatorFollowerConfig.idleMode(IdleMode.kBrake);
    elevatorFollowerConfig.voltageCompensation(12);

    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    elevator.setCANTimeout(0);
    elevatorFollower.setCANTimeout(0);
  }

  @Override
  public void periodic() {
    // Impact of gravity changes with elevator angle
    elevatorFeedforward.setKg(ELEVATOR_KG * Math.sin(pivotAngleRadSupplier.getAsDouble()));
    pid.setPeriod(Math.max(0.02, Timer.getFPGATimestamp() - lastTimestamp));
    lastTimestamp = Timer.getFPGATimestamp();
    double ff = 0, pidOutput = 0;
    switch (mode) {
      case PROFILED_PID:
        ff = elevatorFeedforward.calculate(profiledPid.getSetpoint().velocity);
        pidOutput = profiledPid.calculate(getExtension());
        break;
      case TRAJECTORY:
        ff = elevatorFeedforward.calculate(sample.elevatorVel(), sample.elevatorAccel());
        double setpoint = sample.elevatorLen()-MIN_ABSOLUTE_HEIGHT;
        setpoint = MathUtil.clamp(setpoint, MIN_RELATIVE_HEIGHT, MAX_RELATIVE_HEIGHT);
        pidOutput = pid.calculate(getExtension(), setpoint);
        break;
      case VOLTAGE:
        pidOutput = manualVoltage;
        break;
    }


    Logger.recordOutput("Telescope/ffVolts", ff);
    Logger.recordOutput("Telescope/pidVolts", pidOutput);
    double output = ff + pidOutput;

    if(mode != RunMode.PROFILED_PID || DriverStation.isDisabled()) {
      profiledPid.reset(getExtension());
    } 

    if(mode != RunMode.TRAJECTORY || DriverStation.isDisabled()) {
      pid.reset();
    }

    SmartDashboard.putData("Telescope/pidController", pid);
    Logger.recordOutput("Telescope/attemptedOutput", output);
    Logger.recordOutput("Telescope/manualVoltage", manualVoltage);
    Logger.recordOutput("Telescope/isHomed", isHomed);

    Logger.recordOutput("Telescope/NoOutputReason", "");

    // stops robot from runnign into itself
    if (output > 0 && (!isHomed || getExtension() > MAX_RELATIVE_HEIGHT)) {
      output = 0;
      Logger.recordOutput("Telescope/NoOutputReason", "Not homed or too far up, +attempted");
    }

    if (isHomed == true && (output < 0 && getExtension() < MIN_RELATIVE_HEIGHT)) {
      output = 0;
      Logger.recordOutput("Telescope/NoOutputReason", "homed, too far down -attempted");
    }

    if (!isHomed) output = MathUtil.clamp(output, -2, 2);

    if (mode == RunMode.PROFILED_PID) {
      if (profiledPid.getSetpoint().position < MIN_RELATIVE_HEIGHT || profiledPid.getSetpoint().position > MAX_RELATIVE_HEIGHT) {
        Logger.recordOutput("Telescope/prof pid setpoint too high", "profiled pid setpoint too high");
        // Might as well just get as close as we can
        // dont even work lmao
        // if (profiledPid.getGoal().position < MIN_RELATIVE_HEIGHT || profiledPid.getGoal().position > MAX_RELATIVE_HEIGHT) {
        //   profiledPid.setGoal(MathUtil.clamp(profiledPid.getGoal().position, MIN_RELATIVE_HEIGHT, MAX_RELATIVE_HEIGHT));
        // }
      }
    }

   if (getLimitSwitch()) {
      setEncoderPosition(0);
      isHomed = true;
    }


    Logger.recordOutput("Telescope/output", output);
    setMotorVoltage(output);
  }

  @AutoLogOutput
  public boolean getLimitSwitch() {
    return elevator.getReverseLimitSwitch().isPressed(); 
  }

  public void setGoalPosition(double setpointLength) {
    profiledPid.setGoal(setpointLength);
    pid.setSetpoint(setpointLength);
  }

  public void resetProfiledPidTo(double length) {
    profiledPid.reset(length, 0);
  }

  public void setSample(ArmvatorSample sample) {
    this.sample = sample;
  }

  public void setMode(RunMode mode) {
    // todo: handle some state changes
    this.mode = mode;
  }
  
  @AutoLogOutput
  public RunMode getMode() {
    return mode;
  }

  @AutoLogOutput
  public double getExtension() {
    return elevatorEncoder.getPosition();
  }

  public Command deHome() {
    return runOnce(()->{isHomed = false;});
  }

  @AutoLogOutput
  public double getGoalExtension() {
    if (mode == RunMode.TRAJECTORY) {
      return pid.getSetpoint();
    } else if (mode == RunMode.PROFILED_PID) {
      return profiledPid.getGoal().position;
    } else {
      return getExtension();
    }
  }

  @AutoLogOutput
  public double getProfiliedPidSetpoint() {
    return profiledPid.getSetpoint().position;
  }

  @AutoLogOutput
  public double getTotalLength() {
    return getExtension() + MIN_ABSOLUTE_HEIGHT;
  }

  @AutoLogOutput
  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  protected void setMotorVoltage(double voltage) {
    elevator.setVoltage(voltage);
  }

  @AutoLogOutput
  public boolean atPidGoal() {
    switch(mode) {
      case PROFILED_PID:
        profiledPid.setTolerance(ELEVATOR_LENGTH_TOLERANCE);
        return profiledPid.atGoal();
      case TRAJECTORY:
        pid.setTolerance(ELEVATOR_LENGTH_TOLERANCE);
        return pid.atSetpoint();
      case VOLTAGE:
        return true;
      default:
        return false; // what
    }
  }

  @AutoLogOutput
  public boolean atTrajStartSetpoint() {
    switch(mode) {
      case PROFILED_PID:
        profiledPid.setTolerance(ELEVATOR_LENGTH_TRAJ_START_TOLERANCE);
        return profiledPid.atGoal();
      case TRAJECTORY:
        pid.setTolerance(ELEVATOR_LENGTH_TRAJ_START_TOLERANCE);
        return pid.atSetpoint();
      case VOLTAGE:
        return true;
      default:
        return false; // what
    }
  }

  public void setVoltage(double volts) {
    manualVoltage = volts;
  }

  public void setVoltage(Voltage volts) {
    manualVoltage = volts.magnitude();
  }

  public void setEncoderPosition(double position) {
    elevatorEncoder.setPosition(position);
  }

  public Command setModeTemporarilyTo(RunMode newMode) {
    // intentionally does not require this subsystem
    RunMode originalMode = mode;
    return Commands.startEnd(() -> {
      setMode(newMode);
    }, () -> {
      setMode(originalMode);
    });
  }

  public Command getHomingRoutine() {
    return Commands.sequence(
      this.run(() -> {setVoltage(HOMING_VOLTAGE);}).until(this::getLimitSwitch),
      this.run(() -> {setVoltage(-HOMING_VOLTAGE/2.0);}).until(()->!getLimitSwitch())
    ).deadlineFor(setModeTemporarilyTo(RunMode.VOLTAGE));
  }

  public Command homeIfNeeded() {
    return this.defer(() ->{
      if (!isHomed) {
        return getHomingRoutine();
      } else {
        return this.runOnce(()->{});
      }
    });
  }

  public SysIdRoutine getSysidRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.3).per(Seconds), Volts.of(3), null,
          (state) -> Logger.recordOutput("Telescope/SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism((Voltage v) -> {
        setMode(RunMode.VOLTAGE);
        setVoltage(v);
      }, null, this)
    );
  }

  public boolean getSysidForwardLimit() {
    return getExtension() > MAX_SYSID_HEIGHT;
  }

  public boolean getSysidReverseLimit() {
    return getExtension() < MIN_SYSID_HEIGHT;
  }

  public void setBrakeCoast(boolean willBrake) {
    elevator.setCANTimeout(250);
    elevatorFollower.setCANTimeout(250);

    elevatorConfig.idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    elevatorFollowerConfig.idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    elevator.configureAsync(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorFollower.configureAsync(elevatorFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    elevator.setCANTimeout(0);
    elevatorFollower.setCANTimeout(0);
    Logger.recordOutput("Telescope/isBraken", willBrake);
  }
}
