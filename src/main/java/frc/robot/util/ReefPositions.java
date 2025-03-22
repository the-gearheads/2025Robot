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
import frc.robot.subsystems.vision.Vision;

public class ReefPositions {

  private static final int[] REEF_TAG_IDS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private static final List<Pose2d> REEF_TAG_POSES = initalizeReefTagPoses(REEF_TAG_IDS);
  
  // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
  private static final Translation2d REEF_CENTER_BLUE = Vision.field.getTagPose(18).get().toPose2d().getTranslation()
      .plus(Vision.field.getTagPose(21).get().toPose2d().getTranslation()).div(2);

  // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
  private static final Translation2d REEF_CENTER_RED = Vision.field.getTagPose(10).get().toPose2d().getTranslation()
      .plus(Vision.field.getTagPose(7).get().toPose2d().getTranslation()).div(2);

  private static boolean flipToRed; // whether to use red reef (otherwise blue)

  // Distance from center of robot to center of reef
  // Found by taking distance from tag 18 to center and adding offset from reef
  // private static final Distance REEF_APOTHEM = Meters.of(
  //         Vision.field.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
  //         .plus(Meters.of(MiscConstants.DISTANCE_TO_REEF));
  // private static final Distance REEF_APOTHEM = Meters.of(
  //         Vision.field.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
  //         .plus(Meters.of(MiscConstants.DISTANCE_TO_REEF));
  private static final Distance REEF_APOTHEM = Inches.of(33.75 + 15.75 + 2);
// 33 .75 + 15.75
  // translation to move from centered on a side to scoring position for the left branch
  private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
          Inches.of(6.5));

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

  public static List<Pose2d> getScoringPoses() {
    ArrayList<Pose2d> out = new ArrayList<>();
    for (int i = 0; i < 8; i++) { 
      out.add(getReefPose(i, -1));
      out.add(getReefPose(i, 1));
    }
    return out;
  }

  public static List<Pose2d> getCenterPoses() {
    ArrayList<Pose2d> out = new ArrayList<>();
    for (int i = 0; i < 8; i++) { 
      out.add(getReefPose(i, 0));
    }
    return out;
  }

  private static Pose2d flipPose(Pose2d pose) {
      Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
      Translation2d poseTranslation = pose.getTranslation();
      poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
      return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
  }

  public static int getClosestReefTagId(Pose2d pose) {
    return REEF_TAG_IDS[REEF_TAG_POSES.indexOf(pose.nearest(REEF_TAG_POSES))];
  }

  public static Pose2d getClosestReefTagPose(Pose2d pose) {
    return pose.nearest(REEF_TAG_POSES);
  }

  private static List<Pose2d> initalizeReefTagPoses(int[] reefTagIds) {
    ArrayList<Pose2d> out = new ArrayList<>();
    for (int id : reefTagIds) {
      out.add(Vision.field.getTagPose(id).get().toPose2d());
    }
    return out;
  }
}
