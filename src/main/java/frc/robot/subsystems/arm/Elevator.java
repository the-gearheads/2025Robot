package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  SparkFlex elevator = new SparkFlex(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  SparkFlex elevatorFollower = new SparkFlex(ELEVATOR_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
  SparkFlexConfig elevatorConfig = new SparkFlexConfig();
  SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();

  RelativeEncoder elevatorEncoder = elevator.getEncoder();
  ProfiledPIDController elevatorPid = new ProfiledPIDController(ELEVATOR_PID[0], ELEVATOR_PID[1], ELEVATOR_PID[2],
      ELEVATOR_CONSTRAINTS);

  double ff;
  double output;

  public Elevator() {
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

    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevator.setCANTimeout(0);
    elevatorFollower.setCANTimeout(0);
  }
}
