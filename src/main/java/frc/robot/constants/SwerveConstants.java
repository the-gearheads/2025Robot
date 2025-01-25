package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveConstants {
    public static final int[][] MOTOR_IDS = {
      {6, 26}, // FL
      {8, 28}, // FR
      {2, 22}, // BL
      {4, 24}  // BR
    };

    public static final Translation2d[] WHEEL_POSITIONS = {
      new Translation2d(0.3305, 0.3313), // TODO: placeholders, note that +x is forward and +y is left
      new Translation2d(0.3305, -0.3313),
      new Translation2d(-0.3305, 0.3313),
      new Translation2d(-0.3305, -0.3313)
    };

    // ----------------- TODO: everything here is a placeholder 
    public static final double WHEEL_BASE = WHEEL_POSITIONS[0].getX() - WHEEL_POSITIONS[2].getX();
    public static final double TRACK_WIDTH = WHEEL_POSITIONS[0].getY() - WHEEL_POSITIONS[1].getY();

    public static final double[] WHEEL_OFFSETS = {0, 0, 0, 0};
    public static final boolean[] IS_INVERTED = {true, false, true, false};

    public static final double WHEEL_DIAMETER = 0.073800;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    public static final double DRIVE_PINION_TOOTH_COUNT = 14;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_RATIO = (45.0 * 22) / (DRIVE_PINION_TOOTH_COUNT * 15);

    public static final double ODOMETRY_FREQUENCY = 50;

    // Throughbore encoder is directly on the output steer shaft
    public static final double STEER_RATIO = 1;

    public static final double DRIVE_POS_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO; // rotations -> gear ratio adjusted rotations -> meters
    public static final double DRIVE_VEL_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static final double DRIVE_FREE_RPM = 6784;
    public static final double DRIVE_FREE_SPD = DRIVE_FREE_RPM * DRIVE_VEL_FACTOR; // Convert max neo free speed to max free wheel speed

    public static final double STEER_POS_FACTOR = 2 * Math.PI; // rotations -> radians
    public static final double STEER_VEL_FACTOR = (2 * Math.PI) / 60.0; // rpm -> rad/sec

    public static final double[] STEER_PIDF = {1, 0, 0, 0}; // apparently just a P value of 1 worked for us??? i wanna test that a bit more throughly
    public static final double[] DRIVE_PID = {0.04, 0, 0};
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.25521, 2.0821, 0.10605);

    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int STEER_CURRENT_LIMIT = 20;
    
    public static final double MAX_MOD_SPEED = DRIVE_FREE_SPD;  // m/s, placeholders
    public static final double MAX_ROBOT_TRANS_SPEED = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0.1); // m/s
    public static final double MAX_ROBOT_ROT_SPEED = MAX_ROBOT_TRANS_SPEED / 0.4585738763; // rad/s, 0.45 is radius of robot, spd/r is rad/s

    public static final double HEADING_CONTROLLER_TOLERANCE = 0.005;  // rad

}
