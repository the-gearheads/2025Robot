package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

// This class is autogenerated from avtrajopt/generateungeneratedtrajs.py. Do not edit.
public enum ArmvatorPosition {
  L1(new Translation2d(0.508, 0.95)),
  L2(new Translation2d(0.01, 0.92)),
  L3(new Translation2d(0.01, 1.6)),
  L4(new Translation2d(0.01, 1.8)),
  HP(new Translation2d(0.508, 1)),
  GROUND_INTAKE(new Translation2d(0.508, 0.92)),
  PROCESSOR(new Translation2d(0.508, 0.8)),
  NET(new Translation2d(0.01, 1.942));

  public final Translation2d endeffPos;

  private ArmvatorPosition(Translation2d endeffPos) {
    this.endeffPos = endeffPos;
  }

  public static ArmvatorPosition getNearest(Translation2d endeffPos) {
    ArmvatorPosition best = null;
    double bestDist = Double.POSITIVE_INFINITY;
    for (ArmvatorPosition pos : ArmvatorPosition.values()) {
      double dist = pos.endeffPos.getDistance(endeffPos);
      if (dist < bestDist) {
        best = pos;
        bestDist = dist;
      }
    }
    return best;
  }
}
