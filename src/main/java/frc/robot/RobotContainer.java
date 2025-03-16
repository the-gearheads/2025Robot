// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Teleop;
import frc.robot.commands.NTControl.PivotNTControl;
import frc.robot.commands.NTControl.TelescopeNTControl;
import frc.robot.commands.NTControl.WristNTControl;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.MechanismViz;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructurePosition;
import frc.robot.subsystems.Superstructure.ScoringMode;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.PivotSim;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.TelescopeSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSim;
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
  private final Intake intake;
  private final Autos autos;
  private final SysidAutoPicker sysidAuto = new SysidAutoPicker();
  private final ObjectiveTracker tracker;
  private final MechanismViz viz;
  private final Leds leds = new Leds();

  public RobotContainer() {
    if (Robot.isReal()) {
      pivot = new Pivot();
      telescope = new Telescope();
      wrist = new Wrist();
      intake = new Intake();
    } else {
      pivot = new PivotSim();
      telescope = new TelescopeSim();
      wrist = new WristSim();
      intake = new IntakeSim();
    }
    tracker = new ObjectiveTracker(swerve);
    superStructure = new Superstructure(pivot, telescope, wrist);
    autos = new Autos(swerve, superStructure, intake);
    viz = new MechanismViz(swerve, pivot, telescope, wrist);
    // swerve.setDefaultCommand(new AlignToPose(swerve, tracker::getCoralObjective));
    swerve.setDefaultCommand(new Teleop(swerve));
    // pivot.setDefaultCommand(new ManualPivot(pivot));
    pivot.setDefaultCommand(new PivotNTControl(pivot));
    telescope.setDefaultCommand(telescope.homeIfNeeded().andThen(new TelescopeNTControl(telescope)));
    // telescope.setDefaultCommand(telescope.run(() -> {telescope.setVoltage(0);}));
    wrist.setDefaultCommand(new WristNTControl(wrist));
    // wrist.setDefaultCommand(wrist.run(() -> {wrist.setVoltage(0);}));

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

    // Controllers.driverController.getRightBumper().whileTrue(intake.runIntake());

    // Controllers.driverController.getYBtn().onTrue(new PivotNTControl(pivot));
    // Controllers.driverController.getBBtn().onTrue(new ManualPivot(pivot));
    // teleop controlls


    Controllers.driverController.getYBtn().onTrue(
      Commands.deferredProxy(() -> {
        if (superStructure.getScoringMode() == ScoringMode.CORAL) {
          return superStructure.goTo(SuperstructurePosition.L4);
        } else {
          return superStructure.goTo(SuperstructurePosition.AlgaeL3);
        }
      })
    );

    Controllers.driverController.getBBtn().onTrue(
      Commands.deferredProxy(() -> {
        if (superStructure.getScoringMode() == ScoringMode.CORAL) {
          return superStructure.goTo(SuperstructurePosition.L3);
        } else {
          return superStructure.goTo(SuperstructurePosition.AlgaeL3);
        }
      })
    );

    Controllers.driverController.getABtn().onTrue(
      Commands.deferredProxy(() -> {
        if (superStructure.getScoringMode() == ScoringMode.CORAL) {
          return superStructure.goTo(SuperstructurePosition.L1);
        } else {
          return superStructure.goTo(SuperstructurePosition.AlgaeL2);
        }
      })
    );

    Controllers.driverController.getXBtn().onTrue(
      Commands.deferredProxy(() -> {
        if (superStructure.getScoringMode() == ScoringMode.CORAL) {
          return superStructure.goTo(SuperstructurePosition.L2);
        } else {
          return superStructure.goTo(SuperstructurePosition.AlgaeL2);
        }
      })
    );

    Controllers.driverController.getLeftTriggerBtn().whileTrue(
      intake.runIntake().andThen(
        Commands.deferredProxy(() -> {
          if (superStructure.getScoringMode() == ScoringMode.CORAL) {
            return intake.stop();
          } else {
            return intake.runIntake().repeatedly();
          }
        })
      )
    );
    Controllers.driverController.getRightTriggerBtn().onTrue(
      intake.runOuttake()
    );

    Controllers.driverController.getBackButton().onTrue(
      superStructure.goTo(SuperstructurePosition.L2)
    );

    Controllers.driverController.getStartButton().onTrue(
      superStructure.goTo(SuperstructurePosition.L1)
    );

    Controllers.driverController.getLeftPaddle().onTrue(
      Commands.runOnce(() -> {superStructure.setScoringMode(ScoringMode.ALGAE);})
    );

    Controllers.driverController.getRightPaddle().onTrue(
      Commands.runOnce(() -> {superStructure.setScoringMode(ScoringMode.CORAL);})
    );

    Controllers.driverController.getRightBumper().onTrue(
      superStructure.goTo(SuperstructurePosition.NET)
    );

    Controllers.driverController.getLeftBumper().onTrue(
      Commands.deferredProxy(() -> {
        if (superStructure.getScoringMode() == ScoringMode.CORAL) {
          return superStructure.goTo(SuperstructurePosition.HP);
        } else {
          return superStructure.goTo(SuperstructurePosition.GROUND_INTAKE);
        }
      })
    );
    // Controllers.driverController.getLeftBumper().onTrue(Commands.runOnce(() -> { swerve.setPose(new Pose2d(1, 1, Rotation2d.fromDegrees(0))); }));
    // Controllers.driverController.getRightBumper().whileTrue(new AlignToPose(swerve));
    // Controllers.driverController.getPovLeft().whileTrue(Commands.runEnd(() -> {intake.setVoltage(-12);}, ()->{intake.setVoltage(0);}, intake));
    // Controllers.driverController.getPovRight().whileTrue(Commands.runEnd(() -> {intake.setVoltage(12);}, ()->{intake.setVoltage(0);}, intake));

    Controllers.driverController.getPovLeft().whileTrue(intake.runIntake());
    Controllers.driverController.getPovRight().whileTrue(intake.runOuttake());
    Controllers.driverController.getPovUp().whileTrue(intake.runOuttake(6));
    
    // Controllers.driverController.getLeftBumper().whileTrue(new AlignToPose(swerve, tracker::getCoralObjective));
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
    // return Swerve.wheelRadiusCharacterization(swerve);
  }

  public double getCurrentDrawSim() {
    return swerve.getCurrentDraw();
  }

  public void setAllBrakeCoast(boolean willBrake) {
    pivot.setBrakeCoast(willBrake);
    telescope.setBrakeCoast(willBrake);
    swerve.setBrakeCoast(willBrake);
    wrist.setBrakeCoast(willBrake);
  }
}
