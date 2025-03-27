package frc.robot.subsystems;


import static frc.robot.constants.WristConstants.MAX_WRIST_ANGLE;
import static frc.robot.constants.WristConstants.WRIST_ESCAPE_ANGLE_TOLERANCE;

import java.util.HashMap;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.WristTrajFollower;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ArmvatorPosition;
import frc.robot.util.ArmvatorSample;
import frc.robot.util.ArmvatorTrajectory;

public class Superstructure {
  public static enum RunMode {
    PROFILED_PID, VOLTAGE, TRAJECTORY
  }

  Pivot pivot;
  Telescope telescope;
  Wrist wrist;
  
  HashMap<ArmvatorPosition, Double[]> wristSafeExitAngles = new HashMap<>();

  ArmvatorSample lastSample = new ArmvatorSample(0, 0, 0, 0, 0, 0, 0, 0);
  ArmvatorTrajectory lastTraj = new ArmvatorTrajectory("Aaa", List.of());
  public Superstructure(Pivot pivot, Telescope telescope, Wrist wrist) {
    this.pivot = pivot;
    this.telescope = telescope;
    this.wrist = wrist;
    ArmvatorTrajectory.loadAll();
    // Log for the first time to avoid performance penalty
    Logger.recordOutput("Superstructure/Sample", new ArmvatorSample(0, 0, 0, 0, 0, 0, 0, 0));
    initalizeWristExitAngles();
    telescope.setPivotAngleRadSupplier(pivot::getAngleRad);
  }
  
  private void followSample(ArmvatorSample sample) {
    Logger.recordOutput("Superstructure/Sample", sample);
    pivot.setMode(RunMode.TRAJECTORY);
    telescope.setMode(RunMode.TRAJECTORY);
    pivot.setSample(sample); 
    telescope.setSample(sample);
    lastSample = sample;
  }
  
  @AutoLogOutput
  private boolean atPidEndSetpoint() {
    boolean atSetpoint = pivot.atPidSetpoint() && telescope.atPidSetpoint() && wrist.atPidSetpoint();
    if(pivot.getMode() == RunMode.TRAJECTORY) {
      boolean atEnd = getLastSample().t() >= lastTraj.getDuration();
      return atSetpoint && atEnd;
    }
    return atSetpoint;
  }

  @AutoLogOutput
  private boolean atPidStartSetpoint() {
    return pivot.atTrajStartSetpoint() && telescope.atPidSetpoint();
  }
  
  public Command followAvTrajectory(ArmvatorTrajectory traj) {   
    return traj.follow(this::followSample, this::atPidStartSetpoint, this::atPidEndSetpoint, true, true, pivot, telescope);
  }
  
  public Command goTo(SuperstructurePosition pos) {
    return Commands.defer(()-> {
      var currentPos = ArmvatorPosition.getNearest(getEndEffPos());
      var targetEndeffPos = currentPos.endeffPos;
      var elevatorLength = targetEndeffPos.getNorm();
      var pivotAngle = Math.atan2(targetEndeffPos.getY(), targetEndeffPos.getX());

      Logger.recordOutput("Superstructure/goToFrom", currentPos);
      Logger.recordOutput("Superstructure/goToTo", pos);
        
      // check if wrist needs to move:
      Command wristMoveCommand = Commands.none();
      if (wristSafeExitAngles.containsKey(currentPos)) {
        Double[] safeAngles = wristSafeExitAngles.get(currentPos);
        if (wrist.getAngle().getRadians() < safeAngles[0]) {
          wristMoveCommand = wrist.goTo(Rotation2d.fromRadians(safeAngles[0]), WRIST_ESCAPE_ANGLE_TOLERANCE);
        } else if (wrist.getAngle().getRadians() > safeAngles[1]) {
          wristMoveCommand = wrist.goTo(Rotation2d.fromRadians(safeAngles[1]), WRIST_ESCAPE_ANGLE_TOLERANCE);
        }
      }

      return wristMoveCommand.andThen(Commands.sequence(
        pivot.runOnce(()->{pivot.setMode(RunMode.PROFILED_PID); pivot.setGoalAngle(pivotAngle);}),
        telescope.runOnce(()->{telescope.setMode(RunMode.PROFILED_PID); telescope.setGoalPosition(elevatorLength);}),
        wrist.runOnce(()->{wrist.setGoal(pos.wristAngle);})
      ));
    }, Set.of(pivot, telescope, wrist));
  }
    
    @AutoLogOutput
    public Translation2d getEndEffPos() {
      double x = telescope.getTotalLength() * Math.cos(pivot.getAngle().getRadians());
      double y = telescope.getTotalLength() * Math.sin(pivot.getAngle().getRadians());
      return new Translation2d(x, y);
    }
    
    public ArmvatorSample getLastSample() {
      return lastSample;
    }
    
    @AutoLogOutput
    public ArmvatorPosition getClosestArmvatorPositionEndeff() {
      return ArmvatorPosition.getNearest(getEndEffPos());
    }

    @AutoLogOutput
    public ArmvatorPosition getClosestArmvatorPosition() {
      return ArmvatorPosition.getNearestStateSpace(getEndEffPos());
    }

    public Command waitUntilAtSetpoint() {
      return Commands.waitUntil(()->{
        return this.atPidEndSetpoint() && pivot.getMode() == RunMode.PROFILED_PID;
      });
    }

    public void initalizeWristExitAngles() {
      wristSafeExitAngles.put(ArmvatorPosition.L4, new Double[]{Units.degreesToRadians(5), MAX_WRIST_ANGLE});
    }
  }
