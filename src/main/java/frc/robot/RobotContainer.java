// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.ManualTelescope;
import frc.robot.commands.Teleop;
import frc.robot.commands.NTControl.TelescopeNTControl;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.MechanismViz;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.PivotSim;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.TelescopeSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristSim;
import frc.robot.util.ObjectiveTracker;

public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Swerve swerve = new Swerve();
  private final Pivot pivot;
  private final Telescope telescope;
  private final Wrist wrist;
  private final Superstructure superStructure;
  private final Autos autos = new Autos(swerve);
  private final SysidAutoPicker sysidAuto = new SysidAutoPicker();
  private final ObjectiveTracker tracker;
  private final MechanismViz viz;
  private final Leds leds = new Leds();

  public RobotContainer() {
    if (Robot.isReal()) {
      pivot = new Pivot();
      telescope = new Telescope();
      wrist = new WristSim();
    } else {
      pivot = new PivotSim();
      telescope = new TelescopeSim();
      wrist = new WristSim();
    }
    tracker = new ObjectiveTracker(swerve);
    superStructure = new Superstructure(pivot, telescope, wrist);
    viz = new MechanismViz(swerve, pivot, telescope, wrist);
    // swerve.setDefaultCommand(new AlignToPose(swerve, tracker::getCoralObjective));
    swerve.setDefaultCommand(new Teleop(swerve));
    pivot.setDefaultCommand(new ManualPivot(pivot));
    telescope.setDefaultCommand(new ManualTelescope(telescope));

    sysidAuto.addSysidRoutines("Swerve", swerve.getDriveSysIdRoutine());
    sysidAuto.addSysidRoutines("Swerve Angular", swerve.getAngularSysIdRoutine());
    sysidAuto.addSysidRoutines("Pivot", pivot.getSysidRoutine(), pivot::forwardSysidLimit, pivot::reverseSysidLimit);
    sysidAuto.addSysidRoutines("Wrist", wrist.getSysidRoutine(), wrist::forwardSysidLimit, wrist::reverseSysidLimit);
    sysidAuto.addSysidRoutines("Telescope", telescope.getSysidRoutine(), telescope::getSysidForwardLimit, telescope::getSysidReverseLimit);
  }

  public void configureBindings() {
    if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    Controllers.driverController.getYBtn().onTrue(new TelescopeNTControl(telescope));
    Controllers.driverController.getBBtn().onTrue(new ManualTelescope(telescope));
    // teleop controlls
    // Controllers.driverController.getYBtn().onTrue(
    //   superStructure.goTo(SuperstructurePosition.L4)
    // );

    // Controllers.driverController.getBBtn().onTrue(
    //   superStructure.goTo(SuperstructurePosition.HP)
    // );

    // Controllers.driverController.getLeftBumper().whileTrue(new AlignToPose(swerve, tracker::getCoralObjective));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autos.getAutonomousRoutine();
    return sysidAuto.get();
    // return Swerve.wheelRadiusCharacterization(swerve);
  }

  public double getCurrentDrawSim() {
    return swerve.getCurrentDraw();
  }

  public void setAllBrakeCoast(boolean willBrake) {
    pivot.setBrakeCoast(willBrake);
    telescope.setBrakeCoast(willBrake);
    swerve.setBrakeCoast(willBrake);
  }
}
