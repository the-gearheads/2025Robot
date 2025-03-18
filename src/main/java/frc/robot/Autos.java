package frc.robot;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructurePosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
  Swerve swerve;
  Superstructure superstructure;
  Intake intake;
  AutoFactory factory;
  AutoChooser chooser;

  String nameCenterReef = "Auto Center->Reef Routine";
  String nameLeftReefFeederReef = "Auto Left->Reef->Feeder->Reef Routine";
  String nameRightReefFeederReef = "Auto Right->Reef->Feeder->Reef Routine";

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


    factory.bind("intake", intake.runOnce(()->{}));
    factory.bind("L1", superstructure.goTo(SuperstructurePosition.L1));
    factory.bind("L2", superstructure.goTo(SuperstructurePosition.L2));
    factory.bind("L3", superstructure.goTo(SuperstructurePosition.L3));
    factory.bind("L4", superstructure.goTo(SuperstructurePosition.L4));
    factory.bind("HP", superstructure.goTo(SuperstructurePosition.HP));

    chooser = new AutoChooser();
    chooser.addRoutine(nameCenterReef, this::centerReef);
    chooser.addRoutine(nameLeftReefFeederReef, this::leftReefFeederReef);
    chooser.addRoutine(nameRightReefFeederReef, this::rightReefFeederReef);
    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousRoutine() {
    return chooser.selectedCommand();
  }

   /* Outtake Coral, proxied such that we don't have self-cancelling autons */
  private Command outtakeCoral() {
    return intake.outtakeCoral().asProxy();
  }

  public AutoRoutine centerReef() {
    AutoRoutine routine = factory.newRoutine(nameCenterReef);
    AutoTrajectory trajectory = routine.trajectory("center_reef");

    routine.active().onTrue(
      Commands.sequence(
        trajectory.resetOdometry(),
        trajectory.cmd()
      )
    );

    // Add command to place corral on top level of reef
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag

    return routine;
  }

  public AutoRoutine leftReefFeederReef() {
    AutoRoutine routine = factory.newRoutine(nameLeftReefFeederReef);
    AutoTrajectory trajectoryStartToReefK = routine.trajectory("left_reef_feeder_reef", 0);
    AutoTrajectory trajectoryReefKToFeeder = routine.trajectory("left_reef_feeder_reef", 1);
    AutoTrajectory trajectoryFeederToReefL = routine.trajectory("left_reef_feeder_reef", 2);

    routine.active().onTrue(
      Commands.sequence(
        trajectoryStartToReefK.resetOdometry(),
        trajectoryStartToReefK.cmd()
      )
    );

    // Add command to place corral on top level of reef
    trajectoryStartToReefK.atTime("Start").onTrue(superstructure.goTo(SuperstructurePosition.L4));
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag
    trajectoryStartToReefK.atTime("K-L4").onTrue(intake.outtakeCoral());
    trajectoryStartToReefK.done().onTrue(trajectoryReefKToFeeder.cmd());

    // Resetting the arm to HP position and running the intake at feeder station
    trajectoryReefKToFeeder.atTime("K-L4").onTrue(superstructure.goTo(SuperstructurePosition.HP));
    trajectoryReefKToFeeder.atTime("Feeder").onTrue(intake.runIntake());
    trajectoryReefKToFeeder.done().onTrue(trajectoryFeederToReefL.cmd());

    // Add command to place corral on top level of reef
    trajectoryFeederToReefL.atTime("Feeder").onTrue(superstructure.goTo(SuperstructurePosition.L4));
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag
    trajectoryFeederToReefL.atTime("L-L4").onTrue(intake.outtakeCoral());


    /*
     * 
     * we might want to start intake, then approach the feeder station, and stop intaking some time after driving away perhaps
we want to start to move the armvator into position before we approach the reef, but after we leave the HP station
we need to ensure we're already rotated correctly before lining against stuff like the reef. place like a pose waypoint with the correct rotation some ways out so the robot can get rotationally aligned early
     */

    return routine;
  }

  public AutoRoutine rightReefFeederReef() {
    AutoRoutine routine = factory.newRoutine(nameRightReefFeederReef);
    AutoTrajectory startToReef = routine.trajectory("right_reef_feeder_reef", 0);
    AutoTrajectory reefToHP = routine.trajectory("right_reef_feeder_reef", 1);
    AutoTrajectory HPToReef = routine.trajectory("right_reef_feeder_reef", 2);

    routine.active().onTrue(
      Commands.sequence(
        startToReef.resetOdometry(),
        startToReef.cmd()
      )
    );

    startToReef.done().onTrue(
      Commands.sequence(
        superstructure.waitUntilAtSetpoint(),
        // possibly an auto align
        outtakeCoral().withTimeout(2), // mostly for now as we do not have coral sim yet
        reefToHP.cmd()
      )
    );

    // we -could- hypothetically- wait until we have a game piece, but we could also just rely on pure HP skill
    reefToHP.done().onTrue(Commands.sequence(
      superstructure.waitUntilAtSetpoint(),
      HPToReef.cmd()
    ));

    HPToReef.done().onTrue(
      Commands.sequence(
        superstructure.waitUntilAtSetpoint(),
        // possibly an auto align
        outtakeCoral().withTimeout(2)
      )
    );
    
    // NOTE: may need to add an alignment in the case it is not perfectly aligned in relation to the april tag

    return routine;
  }

}