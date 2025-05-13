// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ENABLED;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.Teleop;
import frc.robot.commands.NTControl.PivotNTControl;
import frc.robot.commands.NTControl.TelescopeNTControl;
import frc.robot.commands.NTControl.WristNTControl;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.MechanismViz;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RunMode;
import frc.robot.subsystems.SuperstructurePosition;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.PivotSim;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.TelescopeSim;
import frc.robot.subsystems.intake.GamePiece;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristSim;
import frc.robot.util.ArmvatorPosition;
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
  @SuppressWarnings("unused")
  private final MechanismViz viz;
  @SuppressWarnings("unused")

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
    swerve.setDefaultCommand(new Teleop(swerve, intake, tracker));
    // swerve.setDefaultCommand(new Teleop(swerve));
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

    Controllers.driverController.getLeftTriggerBtn().onTrue(superStructure.goTo(SuperstructurePosition.GROUND_INTAKE));
    Controllers.driverController.getLeftTriggerBtn().whileTrue(intake.runIntake());

    Controllers.driverController.getRightTriggerBtn().whileTrue(intake.runIntake());
    Controllers.driverController.getRightTriggerBtn().onTrue(superStructure.goTo(SuperstructurePosition.HP));

    Controllers.driverController.getRightPaddle().onTrue(
      Commands.deferredProxy(() -> {
      switch(intake.getGamePiece()) {
        case CORAL:
          if (tracker.facingReef()) {
            return superStructure.goTo(SuperstructurePosition.L2);
          } else {
            return superStructure.goTo(SuperstructurePosition.L4);
          }
        case ALGAE:
          return superStructure.goTo(SuperstructurePosition.NET);
        case EMPTY:
        default:
          return superStructure.goTo(SuperstructurePosition.AlgaeL3).alongWith(intake.runIntake());
      }
      })
    );

    Controllers.driverController.getLeftPaddle().onTrue(
      Commands.deferredProxy(() -> {
      switch(intake.getGamePiece()) {
        case CORAL:
          if (tracker.facingReef()) {
            return superStructure.goTo(SuperstructurePosition.L1);
          } else {
            return superStructure.goTo(SuperstructurePosition.L3);
          }
        case ALGAE:
          return superStructure.goTo(SuperstructurePosition.PROCESSOR);
        case EMPTY:
        default:
          return superStructure.goTo(SuperstructurePosition.AlgaeL2).alongWith(intake.runIntake());
      }
      })
    );

    Controllers.driverController.getRightBumper().onTrue(
      Commands.deferredProxy(() -> {
        switch(intake.getGamePiece()) {
          case CORAL:
            return intake.outtakeCoral();
          case ALGAE:
            return wrist.runOnce(() -> {
              if(superStructure.getClosestArmvatorPosition() == ArmvatorPosition.NET) {
                wrist.setGoal(SuperstructurePosition.NET.wristAngle.minus(Rotation2d.fromDegrees(50)));
              }
            }).alongWith(pivot.run(()->{
              pivot.setMode(RunMode.PROFILED_PID);
              pivot.setGoalAngle(Rotation2d.fromDegrees(85));
            }))
            .alongWith(intake.runOuttake(12));
          default:
            return intake.runOuttake(12);
        }
        })
      );

    Controllers.driverController.getLeftBumper().onTrue(
      superStructure.goTo(SuperstructurePosition.STOW)
    );
    // Controllers.driverController.getLeftBumper().onTrue(Commands.runOnce(() -> { swerve.setPose(new Pose2d(1, 1, Rotation2d.fromDegrees(0))); }));
    // Controllers.driverController.getRightBumper().whileTrue(new AlignToPose(swerve));
    // Controllers.driverController.getPovLeft().whileTrue(Commands.runEnd(() -> {intake.setVoltage(-12);}, ()->{intake.setVoltage(0);}, intake));
    // Controllers.driverController.getPovRight().whileTrue(Commands.runEnd(() -> {intake.setVoltage(12);}, ()->{intake.setVoltage(0);}, intake));
    Controllers.driverController.getPovLeft().whileTrue(intake.runIntake());
    Controllers.driverController.getPovRight().whileTrue(intake.runOuttake(12));
    Controllers.driverController.getPovUp().whileTrue(intake.runOuttake(6));


    Controllers.operatorController.getBtn11().onTrue(Commands.runOnce(()->{AUTO_ALIGN_ENABLED = true;}));
    Controllers.operatorController.getBtn12().onTrue(Commands.runOnce(()->{AUTO_ALIGN_ENABLED = false;}));
    Controllers.operatorController.getBtn13().onTrue(Commands.runOnce(()->{VisionConstants.USE_2D_ALIGNMENT_MODE = true;}));

    Controllers.operatorController.getBtn21().whileTrue(intake.forceGamePiece(GamePiece.ALGAE));
    Controllers.operatorController.getBtn22().whileTrue(intake.forceGamePiece(GamePiece.CORAL));
    Controllers.operatorController.getBtn23().whileTrue(intake.forceGamePiece(GamePiece.EMPTY));

    Controllers.driverController.getABtn().whileTrue(
      pivot.run(() -> {pivot.setMode(RunMode.VOLTAGE); pivot.setVoltage(-5);})
        .alongWith(wrist.run(() -> {wrist.setGoal(Rotation2d.fromDegrees(70));}))
        .alongWith(telescope.run(()->{
          telescope.setMode(RunMode.PROFILED_PID);
          telescope.setGoalPosition(ArmConstants.MIN_RELATIVE_HEIGHT);
        })).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );
    Controllers.driverController.getXBtn().whileTrue(
      pivot.run(() -> {pivot.setMode(RunMode.VOLTAGE); pivot.setVoltage(5);})
        .alongWith(wrist.run(() -> {wrist.setGoal(Rotation2d.fromDegrees(70));}))
        .alongWith(telescope.run(()->{
          telescope.setMode(RunMode.PROFILED_PID);
          telescope.setGoalPosition(ArmConstants.MIN_RELATIVE_HEIGHT);
        })).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );
    
    Controllers.driverController.getYBtn().onTrue(
      superStructure.goTo(SuperstructurePosition.NET)
    );

    Controllers.operatorController.getBtn41().onTrue(new InstantCommand(()-> {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        swerve.setPose(new Pose2d(new Translation2d(14.372, 14.372), Rotation2d.kZero));
      } else {
        swerve.setPose(new Pose2d(new Translation2d(3.174, 3.174), Rotation2d.k180deg));
      }
    }));

    Controllers.operatorController.getBtn31().onTrue(Commands.runOnce(wrist::syncIntegratedEncoder).andThen(Commands.runOnce(pivot::syncIntegratedEncoder)));
    Controllers.operatorController.getBtn32().onTrue(telescope.deHome().andThen(telescope.homeIfNeeded()));
    Controllers.operatorController.getBtn33().onTrue(Commands.runOnce(()->{VisionConstants.USE_2D_ALIGNMENT_MODE = false;}));

    // Controllers.driverController.getLeftBumper().whileTrue(new AlignToPose(swerve, tracker::getCoralObjective));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Commands.runOnce(()->{swerve.vision.disable();}).andThen(autos.getAutonomousRoutine());
    return autos.getAutonomousRoutine();
    // return sysidAuto.get();
    // return Swerve.wheelRadiusCharacterization(swerve);
    // return Commands.runOnce(()->{swerve.vision.disable();}).andThen(new InstantCommand(()-> {swerve.setPose(new Pose2d(7.12387752532959 , 7.599511623382568, Rotation2d.kZero));})).andThen(swerve.run(() -> {swerve.drive(new ChassisSpeeds(0.5, 0, 0));}).withTimeout(7));
  }

  public double getCurrentDrawSim() {
    return swerve.getCurrentDraw();
  }

  public void setAllBrakeCoast(boolean willBrake) {
    pivot.setBrakeCoast(willBrake);
    telescope.setBrakeCoast(willBrake);
    swerve.setBrakeCoast(willBrake);
    wrist.setBrakeCoast(willBrake);
    intake.setBrakeCoast(willBrake);
  }
}
