package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
  Swerve swerve;
  AutoFactory factory;
  AutoChooser chooser;

  String nameCenterReef = "Auto Center->Reef Routine";
  String nameLeftReefFeederReef = "Auto Left->Reef->Feeder->Reef Routine";
  String nameRightReefFeederReef = "Auto Right->Reef->Feeder->Reef Routine";

  public Autos(Swerve swerve) {
    this.swerve = swerve;
    factory = new AutoFactory(
      swerve::getPose,
      swerve::setPose,
      swerve::followTrajectory,
      true,
      swerve
    );
    chooser = new AutoChooser();
    chooser.addRoutine(nameCenterReef, this::centerReef);
    chooser.addRoutine(nameLeftReefFeederReef, this::leftReefFeederReef);
    chooser.addRoutine(nameRightReefFeederReef, this::rightReefFeederReef);
    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousRoutine() {
    return chooser.selectedCommand();
  }

  public AutoRoutine centerReef() {
    AutoRoutine routine = factory.newRoutine(nameCenterReef);
    AutoTrajectory lessEpicTraj = routine.trajectory("center_reef");

    routine.active().onTrue(
      Commands.sequence(
        lessEpicTraj.resetOdometry(),
        lessEpicTraj.cmd()
      )
    );
    return routine;
  }

  public AutoRoutine leftReefFeederReef() {
    AutoRoutine routine = factory.newRoutine(nameLeftReefFeederReef);
    AutoTrajectory lessEpicTraj = routine.trajectory("left_reef_feeder_reef");

    routine.active().onTrue(
      Commands.sequence(
        lessEpicTraj.resetOdometry(),
        lessEpicTraj.cmd()
      )
    );

    // Add command to place corral on top level of reef

    // Add command to intake when at feeder station

    // Add command to place corral on top level of reef

    return routine;
  }

  public AutoRoutine rightReefFeederReef() {
    AutoRoutine routine = factory.newRoutine(nameRightReefFeederReef);
    AutoTrajectory lessEpicTraj = routine.trajectory("right_reef_feeder_reef");

    routine.active().onTrue(
      Commands.sequence(
        lessEpicTraj.resetOdometry(),
        lessEpicTraj.cmd()
      )
    );
    
    // Add command to place corral on top level of reef

    // Add command to intake when at feeder station

    // Add command to place corral on top level of reef

    return routine;
  }

}