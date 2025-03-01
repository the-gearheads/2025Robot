package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverController {

  XboxController controller;

  public DriverController(int id) {
    if(id == -1) {
      this.controller = null;
      return;
    }
    this.controller = new XboxController(id);
  }

  private boolean isNull() {
    return controller == null;
  }

  @SuppressWarnings("unused")
  private Trigger emptyTrigger() {
    return new Trigger(() -> false);
  }

  public double getTranslateXAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftY());
  }

  public double getTranslateYAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftX());
  }

  public double getRotateAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getRightX());
  }

  public double getSpeedUpAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getLeftTriggerAxis());
  }
  
  public double getSlowDownAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getRightTriggerAxis());
  }

  public boolean getXBtn() {
    if(isNull()) return false;
    return controller.getXButton();
  }

  public boolean getABtn() {
    if(isNull()) return false;
    return controller.getAButton();
  }

  public Trigger getYBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getYButton());
  }

  public Trigger getBBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getBButton());
  }

  public Trigger getLeftBumper() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftBumperButton());
  }

  public Trigger getRightBumper() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightBumperButton());
  }

  public void setRumble(double rumble) {
    if(isNull()) return;
    controller.setRumble(RumbleType.kBothRumble, rumble);
  }

  public Command getRumbleCommand(double rumble, double seconds) {
    return Commands.runEnd(()->setRumble(rumble), ()->setRumble(0)).withTimeout(seconds);
  }

  public Command getRumbleCommand(double rumble, double seconds, int pulses) {
    final int pulseCounter[] = {0};
    return Commands.runEnd(
      ()->setRumble(rumble),
      ()->{
        setRumble(0);
        pulseCounter[0]++;
      })
      .withTimeout(seconds)
      .andThen(new WaitCommand(seconds*1.2))
      .repeatedly()
      .until(() -> pulseCounter[0]==pulses);
  }
}
