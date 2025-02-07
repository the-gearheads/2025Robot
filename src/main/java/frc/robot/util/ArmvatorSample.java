package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class ArmvatorSample {
  public final double t;
  public final double num;
  public final double armPos;
  public final double armVel;
  public final double elevatorPos;
  public final double elevatorVel;
  public final double armAccel;
  public final double elevatorAccel;

  public ArmvatorSample(
      double time,
      double sampleNum,
      double armPos,
      double armVel,
      double elevatorPos,
      double elevatorVel,
      double armAccel,
      double elevatorAccel) {
    this.t = time;
    this.num = sampleNum;
    this.armPos = armPos;
    this.armVel = armVel;
    this.elevatorPos = elevatorPos;
    this.elevatorVel = elevatorVel;
    this.armAccel = armAccel;
    this.elevatorAccel = elevatorAccel;
  }

  public String toString() {
    return "Time: " + t + " Sample: " + num + " ArmPos: " + armPos + " ArmVel: " + armVel + " ElevatorPos: "
        + elevatorPos + " ElevatorVel: " + elevatorVel + " ArmAccel: " + armAccel + " ElevatorAccel: " + elevatorAccel;
  }

  public ArmvatorSample interpolate(ArmvatorSample other, double alpha) {
    return new ArmvatorSample(
      MathUtil.interpolate(t, other.t, alpha),
      MathUtil.interpolate(num, other.num, alpha),
      MathUtil.interpolate(armPos, other.armPos, alpha),
      MathUtil.interpolate(armVel, other.armVel, alpha),
      MathUtil.interpolate(elevatorPos, other.elevatorPos, alpha),
      MathUtil.interpolate(elevatorVel, other.elevatorVel, alpha),
      MathUtil.interpolate(armAccel, other.armAccel, alpha),
      MathUtil.interpolate(elevatorAccel, other.elevatorAccel, alpha)
    );
  }
}
