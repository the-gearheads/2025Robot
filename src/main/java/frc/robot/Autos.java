package frc.robot;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructurePosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AlignToPose;
import frc.robot.util.ReefPositions;

public class Autos {
  Swerve swerve;
  Superstructure superstructure;
  Intake intake;
  AutoFactory factory;
  AutoChooser chooser;

  public Autos(Swerve swerve, Superstructure superstructure, Intake intake) {
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.intake = intake;
    Logger.recordOutput("Swerve/Traj/Trajectory", new Pose2d[0]);
    factory = new AutoFactory(
      swerve::getPose,
      swerve::setPose,
      swerve::followTrajectory,
      true,
      swerve,
      (traj, isStarting)->{
        if(isStarting) {
          if(DriverStation.getAlliance().get() == Alliance.Red) {
            traj = traj.flipped();
          }
          Logger.recordOutput("Swerve/Traj/Trajectory", traj.getPoses());
        } else {
          Logger.recordOutput("Swerve/Traj/Trajectory", new Pose2d[0]);
        }
      }
    );


    factory.bind("intake", intake.runIntake());
    factory.bind("intakeAlgae", intake.run(() -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE)).until(intake::hasGamePiece));
    factory.bind("L1", superstructureGoTo(SuperstructurePosition.L1));
    factory.bind("L2", superstructureGoTo(SuperstructurePosition.L2));
    factory.bind("L3", superstructureGoTo(SuperstructurePosition.L3));
    factory.bind("L4", superstructureGoTo(SuperstructurePosition.L4));
    factory.bind("HP", superstructureGoTo(SuperstructurePosition.HP));
    factory.bind("AlgaeL2", superstructureGoTo(SuperstructurePosition.AlgaeL2));
    factory.bind("NET", superstructureGoTo(SuperstructurePosition.NET));
    factory.bind("VisionReefAlignMode", Commands.runOnce(()->{
      var vision = swerve.vision;
      int nearestTagId = ReefPositions.getClosestReefTagId(swerve.getPose());
      Rotation2d gyroOffset = swerve.getPose().getRotation().minus(swerve.getPoseWheelsOnly().getRotation());
      AlignToPose.enableReefVision(vision, gyroOffset, nearestTagId);
    }));

    chooser = new AutoChooser();
    chooser.addRoutine("Left 2 Coral", ()->{return twoCoral("LEFT_2C4", false);});
    chooser.addRoutine("Right 2 Coral", ()->{return twoCoral("RIGHT_2C4", false);});
    chooser.addRoutine("Left 2 Coral HP", ()->{return twoCoral("LEFT_2C4", true);});
    chooser.addRoutine("Right 2 Coral HP", ()->{return twoCoral("RIGHT_2C4", true);});
    chooser.addRoutine("Left 3 Coral", ()->{return threeCoral("LEFT_3C4");});
    chooser.addRoutine("Center 1 Coral", ()->{return center1Coral();});
    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousRoutine() {
    return chooser.selectedCommand();
  }

   /* Outtake Coral, proxied such that we don't have self-cancelling autons */
  private Command outtakeCoral() {
    return intake.outtakeCoral(0.25).asProxy();
  }

  private Command waitForCoral() {
    return Commands.waitUntil(intake::hasGamePiece).asProxy();
  }

  /* Go to a superstructure position, proxied such that we don't have self-cancelling autons */
  private Command superstructureGoTo(SuperstructurePosition pos) {
    return superstructure.goTo(pos).asProxy();
  }

  private Command stop() {
    return Commands.runOnce(() -> swerve.drive(new ChassisSpeeds()));
  }

  public AutoRoutine center1Coral() {
    AutoRoutine routine = factory.newRoutine("CENTER_1C4");
    AutoTrajectory trajectory = routine.trajectory("CENTER_1C4");

    routine.active().onTrue(
      Commands.sequence(
        trajectory.resetOdometry(),
        Commands.runOnce(()->swerve.vision.disable()),
        superstructureGoTo(SuperstructurePosition.L4),
        trajectory.cmd()
      )
    );

    trajectory.done().onTrue(
      Commands.sequence(
        stop(),
        // possibly an auto align
        outtakeCoral().withTimeout(2),
        Commands.runOnce(()->swerve.vision.enable())
      )
    );

    // Add command to place corral on top level of reef
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag

    return routine;
  }

  public AutoRoutine centerReefAlgae() {
    AutoRoutine routine = factory.newRoutine("center_reef_algae");
    AutoTrajectory startToL4 = routine.trajectory("center_reef_algae", 0);
    AutoTrajectory L4ToPreAlgae = routine.trajectory("center_reef_algae", 1);
    AutoTrajectory L4ToAlgae = routine.trajectory("center_reef_algae", 2);
    AutoTrajectory AlgaeToBarge = routine.trajectory("center_reef_algae", 3);
    AutoTrajectory BargeToSafe = routine.trajectory("center_reef_algae", 4);


    routine.active().onTrue(
      Commands.sequence(
        Commands.runOnce(()->swerve.vision.disable()),
        startToL4.resetOdometry(),
        startToL4.cmd()
      )
    );

    startToL4.done().onTrue(
      Commands.sequence(
        stop(),
        Commands.print("test"),
        superstructure.waitUntilAtSetpoint().withTimeout(3),
        Commands.print("tes2"),
        outtakeCoral().withTimeout(2),
        Commands.print("test3"),
        L4ToPreAlgae.cmd()
      )
    );

    L4ToPreAlgae.done().onTrue(
      Commands.sequence(
        stop(),
        superstructure.waitUntilAtSetpoint(),
        L4ToAlgae.cmd()
      )
    );

    L4ToAlgae.done().onTrue(
      Commands.sequence(
        stop(),
        AlgaeToBarge.cmd()
      )
    );

    AlgaeToBarge.done().onTrue(
      Commands.sequence(
        stop(),
        superstructure.waitUntilAtSetpoint(),
        outtakeCoral().withTimeout(2.2),
        BargeToSafe.cmd()
      )
    );

    BargeToSafe.done().onTrue(
      Commands.sequence(
        stop(),
        Commands.runOnce(()->swerve.vision.enable()),
        superstructureGoTo(SuperstructurePosition.STOW)
      )
    );

    return routine;
  }

  public AutoRoutine twoCoral(String routineName, boolean hpAtEnd) {
    AutoRoutine routine = factory.newRoutine(routineName);
    AutoTrajectory startToReef = routine.trajectory(routineName, 0);
    AutoTrajectory reefToHP = routine.trajectory(routineName, 1);
    AutoTrajectory HPToReef = routine.trajectory(routineName, 2);

    routine.active().onTrue(
      Commands.sequence(
        // startToReef.resetOdometry(),
        startToReef.cmd()
      )
    );

    startToReef.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignCommand(swerve, swerve.vision).withTimeout(1.5),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(5),
        outtakeCoral().withTimeout(1),
        reefToHP.cmd()
      )
    );

    // we -could- hypothetically- wait until we have a game piece, but we could also just rely on pure HP skill
    reefToHP.done().onTrue(Commands.sequence(
      stop(),
      superstructure.waitUntilAtSetpoint().withTimeout(3),
      waitForCoral().withTimeout(2),
      HPToReef.cmd()
    ));

    Command hpToReefCommand = Commands.sequence(
      stop(),
      AlignToPose.getAutoAlignCommand(swerve, swerve.vision).withTimeout(1.5),
      Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(3),
      outtakeCoral().withTimeout(1.0)
    );

    if(hpAtEnd) {
      hpToReefCommand = hpToReefCommand.andThen(superstructureGoTo(SuperstructurePosition.HP));
    }

    HPToReef.done().onTrue(
      hpToReefCommand
    );
    
    return routine;
  }

  public AutoRoutine threeCoral(String routineName) {
    AutoRoutine routine = factory.newRoutine(routineName);
    AutoTrajectory startToReef = routine.trajectory(routineName, 0);
    AutoTrajectory reefToHP = routine.trajectory(routineName, 1);
    AutoTrajectory HPToReef = routine.trajectory(routineName, 2);
    AutoTrajectory reefToHP2 = routine.trajectory(routineName, 3);
    AutoTrajectory HPToReef2 = routine.trajectory(routineName, 4);

    routine.active().onTrue(
      Commands.sequence(
        startToReef.resetOdometry(),
        startToReef.cmd()
      )
    );

    startToReef.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(1.5),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision)),
        outtakeCoral(),
        reefToHP.cmd()
      )
    );

    // we -could- hypothetically- wait until we have a game piece, but we could also just rely on pure HP skill
    reefToHP.done().onTrue(Commands.sequence(
      stop(),
      superstructure.waitUntilAtSetpoint(),
      waitForCoral().withTimeout(2),
      HPToReef.cmd()
    ));

    HPToReef.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(1.5),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision)),
        outtakeCoral().withTimeout(1.0),
        reefToHP2.cmd()
      )
    );

    reefToHP2.done().onTrue(Commands.sequence(
      stop(),
      superstructure.waitUntilAtSetpoint(),
      waitForCoral().withTimeout(2),
      HPToReef2.cmd()
    ));

    HPToReef2.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(1.5),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision)),
        outtakeCoral().withTimeout(1.0),
        reefToHP2.cmd()
      )
    );

    return routine;
  }
}