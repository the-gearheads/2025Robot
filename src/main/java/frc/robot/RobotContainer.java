// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Teleop;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Swerve swerve = new Swerve();
  private final Autos autos = new Autos(swerve);
  public RobotContainer() {
    swerve.setDefaultCommand(new Teleop(swerve));
  }

  public void configureBindings() {
    if (!Controllers.didControllersChange())
      return;
    
        // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    // teleop controlls
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autos.getAutonomousRoutine();
  }

  public double getCurrentDrawSim() {
    return swerve.getCurrentDraw();
  }
}
