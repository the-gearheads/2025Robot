package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ArmConstants {
  public static final double ARM_LENGTH = Units.inchesToMeters(36);

  public static final int PIVOT_MOTOR_ID = 9;
  public static final int PIVOT_MOTOR_FOLLOWER_ID = 10;
  public static final int PIVOT_ABS_ENCODER_ID = 2;

  public static final double PIVOT_GEAR_RATIO = 500.0/1.0;
  public static final double PIVOT_POS_FACTOR = (1.0 / PIVOT_GEAR_RATIO) * 2 * Math.PI;
  public static final double PIVOT_VEL_FACTOR = ((1.0 / PIVOT_GEAR_RATIO) * 2 * Math.PI) / 60;  // Rad per Sec
  public static final double PIVOT_ABS_ENCODER_OFFSET = (Math.PI / 2);
  public static final int PIVOT_CURRENT_LIMIT = 60;
  public static final double PIVOT_ANGLE_LIVE_FF_THRESHOLD = 10;
 
  public static final double PIVOT_KS = 0.1983;
  public static final double PIVOT_KG = 0.1305;
  public static final double PIVOT_KV = 8.3892;
  public static final double PIVOT_KA = 0.61006;

  public static final double[] PIVOT_PID = {35, 0, 0.8};  // placeholder
  public static final Constraints PIVOT_CONSTRAINTS = new Constraints( // placeholders
    Units.degreesToRadians(150.905432596),
    Units.degreesToRadians(2515)
  );

  public static final double MAX_ANGLE = Units.degreesToRadians(92);
  public static final double MIN_ANGLE = Units.degreesToRadians(13.5);
  public static final double MAX_SYSID_ANGLE = Units.degreesToRadians(80);
  public static final double MIN_SYSID_ANGLE = Units.degreesToRadians(30);

  public static final double PIVOT_ANGLE_TOLERANCE = Units.degreesToRadians(1);

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
  public static final double MIN_ABSOLUTE_HEIGHT  = Units.inchesToMeters(36);

  public static final double MIN_RELATIVE_HEIGHT = Units.inchesToMeters(2);
  public static final double MAX_RELATIVE_HEIGHT = Units.inchesToMeters(76.5) - MIN_ABSOLUTE_HEIGHT;

  public static final double MIN_SYSID_HEIGHT = Units.inchesToMeters(4);
  public static final double MAX_SYSID_HEIGHT = MAX_RELATIVE_HEIGHT - Units.inchesToMeters(4);

  public static final double ELEVATOR_LENGTH_TOLERANCE = Units.inchesToMeters(0.8);

  public static final int ELEVATOR_CURRENT_LIMIT = 80;
  public static final double HOMING_VOLTAGE = -2;
  
  // from 90_adj
  public static final double ELEVATOR_KS = 0.08291;
  public static final double ELEVATOR_KG = 0.14926;
  public static final double ELEVATOR_KV = 8.3753;
  public static final double ELEVATOR_KA = 0.59784;

  public static final double[] ELEVATOR_PID = {30, 0, 0.66};
  public static final Constraints ELEVATOR_CONSTRAINTS = new Constraints(
    6,
    8
  );

}
