// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Teleop;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModuleIOSpark;
import frc.robot.subsystems.swerve.Gyro.GyroIONavX;

public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Swerve swerve = new Swerve(new GyroIONavX(), new SwerveModuleIOSpark(0, "FL"), new SwerveModuleIOSpark(1, "FR"), new SwerveModuleIOSpark(2, "BL"), new SwerveModuleIOSpark(3, "BR"));
  public RobotContainer() {
    swerve.setDefaultCommand(new Teleop(swerve));
    configureBindings();
  }

  private void configureBindings() {
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
    return null;
  }
}
// peepee poopoo
