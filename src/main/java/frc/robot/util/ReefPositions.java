package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.MiscConstants;
import frc.robot.subsystems.vision.Vision;

public class ReefPositions {
  // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
  private static final Translation2d REEF_CENTER_BLUE = Vision.field.getTagPose(18).get().toPose2d().getTranslation()
      .plus(Vision.field.getTagPose(21).get().toPose2d().getTranslation()).div(2);

  // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
  private static final Translation2d REEF_CENTER_RED = Vision.field.getTagPose(10).get().toPose2d().getTranslation()
      .plus(Vision.field.getTagPose(7).get().toPose2d().getTranslation()).div(2);

  private static boolean flipToRed; // whether to use red reef (otherwise blue)

  // Distance from center of robot to center of reef
  // Found by taking distance from tag 18 to center and adding offset from reef
  private static final Distance REEF_APOTHEM = Meters.of(
          Vision.field.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
          .plus(Meters.of(MiscConstants.DISTANCE_TO_REEF));

  // translation to move from centered on a side to scoring position for the left branch
  private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
          Inches.of(12.94 / 2));

  public static Pose2d getReefPose(int side, int relativePos) {
      // determine whether to use red or blue reef position
      flipToRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

      // initially do all calculations from blue, then flip later
      Translation2d reefCenter = REEF_CENTER_BLUE;

      // robot position centered on close reef side
      Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.zero()));
      // translate to correct branch (left, right, center)
      translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos));
      // rotate to correct side
      translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

      // make pose from translation and correct rotation
      Pose2d reefPose = new Pose2d(translation,
              Rotation2d.fromDegrees(-60 * side).plus(Rotation2d.k180deg));

      if (flipToRed) {
          reefPose = flipPose(reefPose);
      }

      return reefPose;
  }

  public static List<Pose2d> getReefPoses(int relativePos) {
    Pose2d[] out = new Pose2d[8];
    for (int i = 0; i < 8; i++) { 
      out[i] = getReefPose(i, relativePos);
    }
    return List.of(out);
  }

  public static List<Pose2d> getReefPoses() {
    ArrayList<Pose2d> out = new ArrayList<>();
    for (int i = 0; i < 8; i++) { 
      out.add(getReefPose(i, -1));
      out.add(getReefPose(i, 1));
    }
    return out;
  }

  private static Pose2d flipPose(Pose2d pose) {
      Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
      Translation2d poseTranslation = pose.getTranslation();
      poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
      return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
  }
}
