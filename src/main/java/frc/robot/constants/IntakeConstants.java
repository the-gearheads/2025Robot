package frc.robot.constants;

public class IntakeConstants {
  public static final int INTAKE_ID = 35;
  public static final int CANANDCOLOR_ID = 2;

  public static final double CORAL_PROXIMITY_THRESHOLD = 0.22;
  public static final double ALGAE_PROXIMITY_THRESHOLD = 0.32;
  
  // unused -- 
  public static final double ALGAE_HUE_THRESHOLD = 0.35;
  public static final double ALGAE_SAT_THRESHOLD = 0.56;
  public static final double ALGAE_VAL_THRESHOLD = 0.05;
  // --

  public static final double INTAKE_GEAR_RATIO = 25.0;
  public static final double INTAKE_POS_FACTOR = 1.0 / INTAKE_GEAR_RATIO;
  public static final double INTAKE_VEL_FACTOR = (1.0 / INTAKE_GEAR_RATIO) / 60;

  public static final int INTAKE_CURRENT_LIMIT = 28;

  public static final double INTAKE_VOLTAGE = 12.0;
  public static final double INTAKE_STALL_VOLTAGE = 1.0;
  public static final double CORAL_OUTTAKE_VOLTAGE = -12.0;

}
