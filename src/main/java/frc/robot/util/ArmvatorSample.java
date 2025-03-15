package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public record ArmvatorSample(
  double t,
  double num,
  double armPos,
  double armVel,
  double elevatorLen,
  double elevatorVel,
  double armAccel,
  double elevatorAccel) {

  public ArmvatorSample interpolate(ArmvatorSample other, double timestamp) {
    double alpha = (timestamp - t) / (other.t - t);
    return new ArmvatorSample(
      MathUtil.interpolate(t, other.t, alpha),
      MathUtil.interpolate(num, other.num, alpha),
      MathUtil.interpolate(armPos, other.armPos, alpha),
      MathUtil.interpolate(armVel, other.armVel, alpha),
      MathUtil.interpolate(elevatorLen, other.elevatorLen, alpha),
      MathUtil.interpolate(elevatorVel, other.elevatorVel, alpha),
      MathUtil.interpolate(armAccel, other.armAccel, alpha),
      MathUtil.interpolate(elevatorAccel, other.elevatorAccel, alpha)
    );
  }
}
