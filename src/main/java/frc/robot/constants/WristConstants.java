package frc.robot.constants;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class WristConstants {
  public static final int WRIST_PIVOT_ID = 36;

  public static final int WRIST_CURRENT_LIMIT = 30;
 
  public static final double WRIST_GEAR_RATIO = 225.0/1.0;
  public static final double WRIST_POS_FACTOR = (1.0/WRIST_GEAR_RATIO) * (2 * Math.PI);
  public static final double WRIST_VEL_FACTOR = ((1.0/WRIST_GEAR_RATIO) * (2 * Math.PI)) / 60;
  public static final double WRIST_ABS_POS_FACTOR = 2 * Math.PI;
  public static final double WRIST_ABS_VEL_FACTOR = (2 * Math.PI) / 60;
  
  public static final double[] WRIST_PID = {9.3, 0, 2};
  public static final ArmFeedforward WRIST_FF = new ArmFeedforward(0.28349, 0.191, 2.0501, 0.32273);
  public static final double WRIST_FF_OFFSET_RAD = 1.4655;
  public static final Constraints WRIST_CONSTRAINTS = new Constraints(4, 3);

  public static final double MIN_WRIST_ANGLE = Units.degreesToRadians(-80);
  public static final double MAX_WRIST_ANGLE = Units.degreesToRadians(120);
  public static final double MIN_SYSID_ANGLE = Units.degreesToRadians(-69);
  public static final double MAX_SYSID_ANGLE = Units.degreesToRadians(105);

  public static final double WRIST_ANGLE_TOLERANCE = Units.degreesToRadians(1);
  public static final double WRIST_ESCAPE_ANGLE_TOLERANCE = Units.degreesToRadians(3); // before running superstrcuture positions

  // all for sim
  public static final double WRIST_MOI_EST = 2; // kg * m^2, VERY rough estimate from cad;
  public static final double WRIST_LENGTH = Units.inchesToMeters(17.0541);
  
}
