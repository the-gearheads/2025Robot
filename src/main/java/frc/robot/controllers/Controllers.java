package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.MiscConstants;

import static edu.wpi.first.math.MathUtil.applyDeadband;

public class Controllers {

  private Controllers() {}

  private static final String[] OPERATOR_CONTROLLERS = {
    "T.16000M",
    "Keyboard 1"
  };
  private static String[] lastControllerNames = new String[DriverStation.kJoystickPorts];
  public static DriverController driverController;
  public static OperatorController operatorController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;
    String name;

    for (int i = 0; i < DriverStation.kJoystickPorts ; i++) {
      name = DriverStation.getJoystickName(i);
      if (!name.equals(lastControllerNames[i])) {
        hasChanged = true;
        lastControllerNames[i] = name;
      }
    }

    return hasChanged;
  }

  public static void updateActiveControllerInstance() {
    boolean foundDriveController = false;
    boolean foundOperatorController = false;
    String joyName;

    driverController = new DriverController(-1);
    operatorController = new OperatorController() {};

    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.isJoystickConnected(port)) {
        joyName = DriverStation.getJoystickName(port);

        if (!foundOperatorController && isOperatorController(joyName)) {
          foundOperatorController = true;
          operatorController = new Thrustmaster(port);
        }
        // No filtering for now, just use the first
        else if (!foundDriveController) {
          foundDriveController = true;
          driverController = new DriverController(port);
        }
      }
    }
  }

  private static boolean isOperatorController(String name) {
    for (String controllerName : OPERATOR_CONTROLLERS) {
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
