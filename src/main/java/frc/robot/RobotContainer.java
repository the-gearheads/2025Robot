// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.ManualTelescope;
import frc.robot.commands.Teleop;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.PivotSim;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.telescopeSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.RunMode;

public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Swerve swerve = new Swerve();
  private final Pivot pivot;
  private final Telescope telescope;
  private final Autos autos = new Autos(swerve);
  private final SysidAutoPicker sysidAuto = new SysidAutoPicker();
  public RobotContainer() {
    if (Robot.isReal()) {
      pivot = new Pivot();
      telescope = new Telescope();
    } else {
      pivot = new PivotSim();
      telescope = new telescopeSim();
    }
    swerve.setDefaultCommand(new Teleop(swerve));
    pivot.setDefaultCommand(pivot.runWithMode(new ManualPivot(pivot), RunMode.VOLTAGE));
    telescope.setDefaultCommand(new ManualTelescope(telescope));
    sysidAuto.addSysidRoutine(swerve.sysIdForwardDynamic(Direction.kForward), "Swerve Dynamic ->");
    sysidAuto.addSysidRoutine(swerve.sysIdForwardQuasistatic(Direction.kForward), "Swerve Quasistatic ->");
    sysidAuto.addSysidRoutine(swerve.sysIdForwardDynamic(Direction.kReverse), "Swerve Dynamic <-");
    sysidAuto.addSysidRoutine(swerve.sysIdForwardQuasistatic(Direction.kReverse), "Swerve Quasistatic <-");
    sysidAuto.addSysidRoutine(swerve.sysIdAngularDynamic(Direction.kForward), "Swerve Angular Dynamic ->");
    sysidAuto.addSysidRoutine(swerve.sysIdAngularQuasistatic(Direction.kForward), "Swerve Angular Quasistatic ->");
    sysidAuto.addSysidRoutine(swerve.sysIdAngularDynamic(Direction.kReverse), "Swerve Angular Dynamic <-");
    sysidAuto.addSysidRoutine(swerve.sysIdAngularQuasistatic(Direction.kReverse), "Swerve Angular Quasistatic <-");

  }

  public void configureBindings() {
    if (!Controllers.didControllersChange())
      return;
    
        // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    // teleop controlls
    Controllers.driverController.getYBtn().whileTrue(new InstantCommand(() -> {
      swerve.zeroGyro();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autos.getAutonomousRoutine();
    // return sysidAuto.get();
  }

  public double getCurrentDrawSim() {
    return swerve.getCurrentDraw();
  }
}
