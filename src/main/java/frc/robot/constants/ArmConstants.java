package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ArmConstants {
  public static final double ARM_LENGTH = Units.inchesToMeters(36);

  public static final int PIVOT_MOTOR_ID = 9;
  public static final int PIVOT_MOTOR_FOLLOWER_ID = 10;

  public static final double PIVOT_GEAR_RATIO = 256.0/1.0;
  public static final double PIVOT_POS_FACTOR = PIVOT_GEAR_RATIO * 2 * Math.PI;
  public static final double PIVOT_VEL_FACTOR = (PIVOT_GEAR_RATIO * 2 * Math.PI) / 60;  // Rad per Sec
  public static final int PIVOT_CURRENT_LIMIT = 60;
  public static final double PIVOT_ANGLE_LIVE_FF_THRESHOLD = 10;

  public static final ArmFeedforward PIVOT_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);  
  public static final double[] PIVOT_PID = {3, 0, 0};  // placeholder
  public static final Constraints PIVOT_CONSTRAINTS = new Constraints( // placeholders
    Units.degreesToRadians(150.905432596),
    Units.degreesToRadians(2515)
  );

  public static final double MAX_ANGLE = Units.degreesToRadians(90);
  public static final double MIN_ANGLE = Units.degreesToRadians(15);
  public static final double MIN_SYSID_ANGLE = Units.degreesToRadians(20);
  public static final double MAX_SYSID_ANGLE = Units.degreesToRadians(85);

  public static final double PIVOT_ANGLE_TOLERANCE = Units.degreesToRadians(0.2);
  public static final double PIVOT_MOI_EST = 0.7895417850; // from cad kg * m^2

  // -- elevator contants --
  public static final int ELEVATOR_MOTOR_ID = 11;
  public static final int ELEVATOR_MOTOR_FOLLOWER_ID = 12;

  public static final double ELEVATOR_GEAR_RATIO = 12;
  public static final double ELEVATOR_DRUM_DIAMETER = 0.98 * 0.0254;
  public static final double ELEVATOR_RATIO = (2 * Math.PI * ELEVATOR_DRUM_DIAMETER) / ELEVATOR_GEAR_RATIO;
  public static final double ELEVATOR_POS_FACTOR = ELEVATOR_RATIO;
  public static final double ELEVATOR_VEL_FACTOR = ELEVATOR_RATIO / 60.0;

  public static final double ELEVATOR_MASS = 13.6078; // mass of elevator carriage (kg)
  public static final double ELEVATOR_RADIUS = 1; // radius of elevator driving drum
  public static final double MIN_RELATIVE_HEIGHT = Units.inchesToMeters(0);
  public static final double MIN_ABSOLUTE_HEIGHT  = Units.inchesToMeters(36);
  public static final double MAX_HEIGHT = Units.inchesToMeters(77.6) - MIN_ABSOLUTE_HEIGHT;

  public static final int ELEVATOR_CURRENT_LIMIT = 80;
  public static final double HOMING_VOLTAGE = -2;
  
  public static final double[] ELEVATOR_PID = {4.2, 0, 0.2};
  public static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(0, 0, 0);
  public static final Constraints ELEVATOR_CONSTRAINTS = new Constraints(
    0.5,
    1
  );

}
