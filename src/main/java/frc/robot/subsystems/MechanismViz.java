package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.swerve.Swerve;

public class MechanismViz extends SubsystemBase {

  private static final boolean doBumpers = true;

  // cad model is off the ground but we want wheels to be on the ground
  private final Transform3d ROBOT_OFFSET = new Transform3d(0, 0, 0.065532, new Rotation3d());
  
  // these are the inverse of the zeroed_position offsets in the ascope model config.json
  private final Transform3d PIVOT_POS = new Transform3d(-0.318, 0, 0.0805, new Rotation3d()).plus(ROBOT_OFFSET);
  private final Transform3d ELEV1_POS = new Transform3d(-0.358, 0, 0.1805, new Rotation3d()).plus(ROBOT_OFFSET);
  private final Transform3d ELEV2_POS = new Transform3d(-0.363, 0, 0.217, new Rotation3d()).plus(ROBOT_OFFSET);
  private final Transform3d WRIST_POS = new Transform3d(-0.363, 0, 0.975, new Rotation3d()).plus(ROBOT_OFFSET);

  // bumpers aren't offset since they dont need to rotate so its dumb to waste effort on that, so 0, 0, 0 is just where they're supposed to be
  private final Transform3d bumperGonePos = new Transform3d(1000, 100, 0, new Rotation3d()); // i sure hope your field isnt 1km

  Swerve swerve;
  Pivot pivot;
  Telescope telescope;
  LoggedNetworkNumber wristAngleInp = new LoggedNetworkNumber("WristAngle", 0);

  public MechanismViz(Swerve swerve, Pivot pivot, Telescope telescope) {
    this.swerve = swerve;
    this.pivot = pivot;
    this.telescope = telescope;
  }

  @Override
  public void periodic() {
    // So our "cad" coordinate frame is going to have 0 degrees as "pointing straight up", which is uhh, not what we use literally everywhere else
    Rotation3d pivotAngle = new Rotation3d(0, Math.toRadians(90) - pivot.getAngle().getRadians(), 0);
    // Rotation3d pivotAngle = new Rotation3d(0, Math.toRadians(0), 0);
    Transform3d stage1Extension = new Transform3d(0, 0, telescope.getPosition()/2.0, new Rotation3d());
    Transform3d stage2Extension = new Transform3d(0, 0, telescope.getPosition()/2.0, new Rotation3d());
    Transform3d totalExtension = stage1Extension.plus(stage2Extension);
    Rotation3d wristAngle = new Rotation3d(0, Math.toRadians(wristAngleInp.get()), 0);

    Transform3d stage0PivotPose = rotateIntrinsically(PIVOT_POS, pivotAngle);
    // We want our extended elevator to extend relative to the pivot, and this gives us the position of the elevator from the pivot
    // a.plus(b.inverse()) is the same as a - b, which is the same as a.relativeTo(b) (which only exists for Pose3d) 
    Transform3d stage1ElevRelativeTo = ELEV1_POS.plus(stage1Extension).plus(PIVOT_POS.inverse());
    // Then we can just add
    Transform3d stage1ElevatorPos = stage0PivotPose.plus(stage1ElevRelativeTo);

    // Ditto for stage2
    Transform3d stage2RelativeTo = ELEV2_POS.plus(totalExtension).plus(PIVOT_POS.inverse());
    Transform3d stage2ElevatorPos = stage0PivotPose.plus(stage2RelativeTo);

    // Pretty much the same for wrist, except we rotate at the end
    Transform3d wristPoseRelativeTo = WRIST_POS.plus(totalExtension).plus(PIVOT_POS.inverse());;
    Transform3d wristPos = rotateIntrinsically(stage0PivotPose.plus(wristPoseRelativeTo), wristAngle);


    // Now lets figure out which bumpers to use. We won't use any if alliance is invalid
    Transform3d bumperBluePos = bumperGonePos, bumperRedPos = bumperGonePos;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && doBumpers) {
      if (alliance.get() == Alliance.Blue) {
        bumperBluePos = new Transform3d(); // they're already zeroed properly, no need to do anything
      } else {
        bumperRedPos = new Transform3d();
      }
    }

    Transform3d[] componentPoses = {stage0PivotPose, stage1ElevatorPos, stage2ElevatorPos, wristPos, bumperBluePos, bumperRedPos};
    Logger.recordOutput("ComponentPoses", componentPoses);
    Logger.recordOutput("Origin", new Pose3d());
  }

  private Transform3d rotateIntrinsically(Transform3d pose, Rotation3d rotation) {
    return new Transform3d(pose.getTranslation(), pose.getRotation().rotateBy(rotation));
  }
}
