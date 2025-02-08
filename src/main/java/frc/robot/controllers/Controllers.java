package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.MiscConstants;

import static edu.wpi.first.math.MathUtil.applyDeadband;

public class Controllers {

  private Controllers() {}

  private static final int MAX_DRIVER_STATION_PORTS = DriverStation.kJoystickPorts; 
  private static final String[] OPERATOR_CONTROLLER_NAMES = {
    "T.16000M",
    "Keyboard 1"
  };
  private static String[] lastControllerNames = new String[MAX_DRIVER_STATION_PORTS];

  public static DriverController driverController;
  public static OperatorController operatorController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;
    String name;

    for (int i = 0; i < MAX_DRIVER_STATION_PORTS ; i++) {
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

  private static boolean isOperatorControllerName(String name) {
    for (String controllerName : OPERATOR_CONTROLLER_NAMES) {
      if (name.contains(controllerName)) {
        return true;
      }
    }
    return false;
  }

  public static double deadband(double num) {
    return applyDeadband(num, MiscConstants.JOYSTICK_DEADBAND);
  }
}
