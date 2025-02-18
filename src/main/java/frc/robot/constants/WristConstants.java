package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class WristConstants {
  public static final int WRIST_PIVOT_ID = 200;

  public static final int WRIST_CURRENT_LIMIT = 60;
 
  public static final double WRIST_GEAR_RATIO = 1;
  public static final double WRIST_POS_FACTOR = WRIST_GEAR_RATIO * 2 * Math.PI;
  public static final double WRIST_VEL_FACTOR = (WRIST_GEAR_RATIO * 2 * Math.PI) / 60;
  
  public static final double[] WRIST_PID = {6, 0, 0};
  public static final ArmFeedforward WRIST_FF = new ArmFeedforward(0, 0, 0, 0);
  public static final Constraints WRIST_CONSTRAINTS = new Constraints(2, 1);

  public static final double MIN_WRIST_ANGLE = Units.degreesToRadians(0);
  public static final double MAX_WRIST_ANGLE = Units.degreesToRadians(180);

  public static final double WRIST_ANGLE_TOLERANCE = Units.degreesToRadians(1);
}
