package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveConstants {
    public static final int[][] MOTOR_IDS = {
        {0, 1}, // TODO: placeholder
        {2, 3}, // placeholder
        {4, 5}, // placeholder
        {6, 7}  // placeholder
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

    public static final double[] WHEEL_OFFSETS = {90, 0, 0, 90};
    public static final boolean[] IS_INVERTED = {true, false, true, false};

    public static final double WHEEL_DIAMETER = 0.073800;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double MODULE_RADIUS = WHEEL_DIAMETER / 2.0;

    // -----------------
}
