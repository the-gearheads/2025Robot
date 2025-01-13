package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {

  public default Trigger getBabyBird() {
    return new Trigger(()->false);
  }

  public default Trigger getIntakeNote() {
    return new Trigger(()->false);
  }

  public default Trigger getArmUp() {
    return new Trigger(()->false);
  }
  
  public default Trigger getArmDown() {
    return new Trigger(()->false);
  }
  
  public default Trigger getArmAutosOff() {
    return new Trigger(()->false);
  }

  public default Trigger getArmAutosOn() {
    return new Trigger(()->false);
  }

  public default Trigger getAmpOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getShooterOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getShooterRevOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getFeederOverride() {
    return new Trigger(()->false);
  }

  public default Trigger getFeederRevOverride() {
    return new Trigger(()->false);
  }
  
  public default Trigger getIntakeOverride() {
    return new Trigger(()->false);
  }
  
  public default Trigger getIntakeRevOverride() {
    return new Trigger(()->false);
  }
  
  public default Trigger getSetAmpModeBtn() {
    return new Trigger(()->false);
  }
  
  public default Trigger getSetSpeakerModeBtn() {
    return new Trigger(()->false);
  }
  
  public default Trigger getSetStageModeBtn() {
    return new Trigger(()->false);
  }

  // public default Trigger getResetPoseBtn() {
  //   return new Trigger(()->false);
  // }

  // public default Trigger getDisableVisionBtn() {
  //   return new Trigger(()->false);
  // }

  public default Trigger climberUp() {
    return new Trigger(()->false);
  }

  public default Trigger climberDown() {
    return new Trigger(()->false);
  }

  public default double getClimberProportion() {
    return 0;
  }
}
