package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;

public class Thrustmaster implements OperatorController {
  Joystick joy;
  public Thrustmaster(int id) {
    joy = new Joystick(id);
  }
  
  // https://ts.thrustmaster.com/download/accessories/Manuals/T16000M/T16000M-User_manual.pdf for button layout

}
