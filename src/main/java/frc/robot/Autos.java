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
    chooser.addRoutine("Left 2 Coral", ()->{return twoCoral("LEFT_2C4");});
    chooser.addRoutine("Right 2 Coral", ()->{return twoCoral("RIGHT_2C4");});
    chooser.addRoutine("Left 3 Coral", ()->{return threeCoral("LEFT_3C4");});
    chooser.addRoutine("Center 1 Coral", ()->{return center1Coral();});
    chooser.addRoutine("test", ()->{return test();});
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

  private Command number(int num) {
    return Commands.runOnce(()->{
      Logger.recordOutput("number", num);
    });
  }

  public AutoRoutine test() {
    AutoRoutine routine = factory.newRoutine("test");
    routine.active().onTrue(
      Commands.sequence(
        swerve.driveToPose(new Pose2d(10.0, 1.628, Rotation2d.kZero)),
        superstructureGoTo(SuperstructurePosition.NET)
      )
    );

    return routine;
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

    return routine;
  }

  public AutoRoutine twoCoral(String routineName) {
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
        number(0),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(2.5),
        number(1),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(3),
        number(2),
        outtakeCoral().withTimeout(1.0),
        reefToHP.cmd()
      )
    );

    // we -could- hypothetically- wait until we have a game piece, but we could also just rely on pure HP skill
    reefToHP.done().onTrue(Commands.sequence(
      stop(),
      superstructure.waitUntilAtSetpoint().withTimeout(0.5),
      waitForCoral().withTimeout(2),
      HPToReef.cmd()
    ));

    HPToReef.done().onTrue(Commands.sequence(
      stop(),
      AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(2.5),
      Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(3),
      outtakeCoral().withTimeout(1.0)
    ));
    
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
        // startToReef.resetOdometry(),
        startToReef.cmd()
      )
    );

    startToReef.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(2.5),
        Commands.print("Finished aligning"),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(3),
        Commands.print("placing coral"),
        outtakeCoral().withTimeout(1.0),
        Commands.print("finished outtaking"),
        reefToHP.cmd()
      )
    );

    // we -could- hypothetically- wait until we have a game piece, but we could also just rely on pure HP skill
    reefToHP.done().onTrue(Commands.sequence(
      stop(),
      superstructure.waitUntilAtSetpoint().withTimeout(2),
      waitForCoral().withTimeout(2),
      HPToReef.cmd()
    ));

    HPToReef.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(2.5),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(3),
        outtakeCoral().withTimeout(1.0),
        reefToHP.cmd()
      )

    );

    reefToHP2.done().onTrue(Commands.sequence(
      stop(),
      superstructure.waitUntilAtSetpoint().withTimeout(2),
      waitForCoral().withTimeout(2),
      HPToReef2.cmd()
    ));

    HPToReef2.done().onTrue(
      Commands.sequence(
        stop(),
        AlignToPose.getAutoAlignEndsCommand(swerve, swerve.vision).withTimeout(2.5),
        Commands.deadline(superstructure.waitUntilAtSetpoint(), AlignToPose.getAutoAlignCommand(swerve, swerve.vision)).withTimeout(3),
        outtakeCoral().withTimeout(1.0),
        reefToHP.cmd()
      )
    );

    return routine;
  }
}