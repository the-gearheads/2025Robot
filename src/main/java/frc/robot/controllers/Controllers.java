package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.MiscConstants;

public class Controllers {

  private Controllers() {}

  private static final int NUMBER_OF_CONTROLLERS = 6;
  private static String[] lastControllerNames = new String[NUMBER_OF_CONTROLLERS];

  public static DriverController driverController;
  public static OperatorController operatorController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;
    String name;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      name = DriverStation.getJoystickName(i);
      if (!name.equals(lastControllerNames[i])) {
        hasChanged = true;
        lastControllerNames[i] = name;
      }
    }

    return hasChanged;
  }

  public static void updateActiveControllerInstance() {
    // Defaults, since a NullPointerException would be far worse than any warnings
    // driverController = new DriverController() {};
    driverController = new DriverController(-1);
    operatorController = new OperatorController() {};

    boolean foundDriveController = false;
    boolean foundOperatorController = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String joyName = DriverStation.getJoystickName(i);
      if (joyName.equals(""))
        continue;


      if (!foundOperatorController && (joyName.contains("T.16000M") || joyName.contains("Keyboard 1"))) {
        foundOperatorController = true;
        operatorController = new Thrustmaster(i);
      }

      // No filtering for now, just use the first
      else if (!foundDriveController) {
        foundDriveController = true;
        driverController = new DriverController(i);
      }
    }
  }

  public static double deadband(double num) {
    return MathUtil.applyDeadband(num, MiscConstants.JOYSTICK_DEADBAND);
  }
}
