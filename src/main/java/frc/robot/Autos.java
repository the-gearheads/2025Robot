package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
    AutoTrajectory trajectory = routine.trajectory("left_reef_feeder_reef");

    routine.active().onTrue(
      Commands.sequence(
        trajectory.resetOdometry(),
        trajectory.cmd()
      )
    );

    // Add command to place corral on top level of reef
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag

    // Add command to intake when at feeder station

    // Add command to place corral on top level of reef
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag

    return routine;
  }

  public AutoRoutine rightReefFeederReef() {
    AutoRoutine routine = factory.newRoutine(nameRightReefFeederReef);
    AutoTrajectory trajectoryStartToReef = routine.trajectory("right_reef_feeder_reef", 0);
    AutoTrajectory trajectoryReefToFromFeeder = routine.trajectory("right_reef_feeder_reef", 1);

    routine.active().onTrue(
      Commands.sequence(
        trajectoryStartToReef.resetOdometry(),
        trajectoryStartToReef.cmd(),
        // put auto align command here
        trajectoryReefToFromFeeder.cmd()
      )
    );
    trajectoryStartToReef.atTime("ArmL4").onTrue(superstructure.goTo(SuperstructurePosition.L4));
    
    // Add command to place corral on top level of reef
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag

    // Add command to intake when at feeder station
    // add a wait??

    // Add command to place corral on top level of reef
    // NOTE: may need to add an "alignment" in the case it is not perfectly aligned in relation to the april tag

    return routine;
  }

}