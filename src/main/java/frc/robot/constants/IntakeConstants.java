package frc.robot.constants;

public class IntakeConstants {
  public static final int INTAKE_ID = 35;
  
  public static final double INTAKE_GEAR_RATIO = 25.0;
  public static final double INTAKE_POS_FACTOR = 1.0 / INTAKE_GEAR_RATIO;
  public static final double INTAKE_VEL_FACTOR = (1.0 / INTAKE_GEAR_RATIO) / 60;

  public static final int INTAKE_CURRENT_LIMIT = 28;

  public static final int INTAKE_VOLTAGE = 12;
  public static final int INTAKE_STALL_VOLTAGE = 1;
  public static final double STALL_VELOCITY_THRESHOLD = 0.25;
}
