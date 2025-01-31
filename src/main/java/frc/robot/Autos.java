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
    chooser.addRoutine("Epic Routine", this::epicRoutine);
    chooser.addRoutine("Less epic routine", this::lessEpicRoutine);
    chooser.addRoutine("Alignment thing", this::testRoutine);
    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousRoutine() {
    return chooser.selectedCommand();
  }

  public AutoRoutine testRoutine() {
    var routine = factory.newRoutine("test path 2");

    AutoTrajectory epicTraj = routine.trajectory("Alignment path");
    routine.active().onTrue(
      Commands.sequence(
        epicTraj.resetOdometry(),
        epicTraj.cmd()
      )
    );
    return routine;  }

  public AutoRoutine epicRoutine() {
    var routine = factory.newRoutine("epic");

    AutoTrajectory epicTraj = routine.trajectory("epic path");
    routine.active().onTrue(
      Commands.sequence(
        epicTraj.resetOdometry(),
        epicTraj.cmd()
      )
    );
    return routine;
  }

  public AutoRoutine lessEpicRoutine() {
    var routine = factory.newRoutine("less epic");
    AutoTrajectory lessEpicTraj = routine.trajectory("less epic path");

    routine.active().onTrue(
      Commands.sequence(
        lessEpicTraj.resetOdometry(),
        lessEpicTraj.cmd()
      )
    );
    return routine;
  }
}