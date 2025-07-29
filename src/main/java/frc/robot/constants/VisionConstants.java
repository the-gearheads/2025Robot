package frc.robot.constants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraIntrinsics;

public class VisionConstants {
  // sane values somewhere between 1e-7 and 10000. low == trust initial guess heading less.
  public static final double CONSTRAINED_PNP_HEADING_SCALE_FACTOR = 1;
  
  public static final String[] CAMERA_NAMES = { "FRONT_LEFT_2025",  "BACK_LEFT_OFFSEASON", "BACK_RIGHT_OFFSEASON"};

  public static final double MAX_AVG_DIST = 2.7;
  public static final double MAX_TAG_AMBIGUITY = 0.3;

  public static boolean USE_2D_ALIGNMENT_MODE = true;
  public static boolean USE_GTSAM_DEFAULT = true; // if this is set to true it will IGNORE all other vision strategies/poses; including 2d alignment mode

  public static final PoseStrategy[] INITAL_CAMERA_STRATEGIES = {PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR};
  public static final Transform3d[] CAMERA_TRANSFORMS = {
      new Transform3d(  // FRONT_LEFT_2025
          new Translation3d(0.36322, 0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0))),
      new Transform3d( // BACK_LEFT_OFFSEASON
          new Translation3d(        -0.1524,
          0.3048,
          0.2286),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-15), Units.degreesToRadians(6.5 - 180))),
      new Transform3d( // BACK_RIGHT_OFFSEASON
      new Translation3d(        -0.1524,
      -0.3048,
      0.2286),
      new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-15), Units.degreesToRadians(-6.5 - 180))),

  };

  // ONLY used for sim; these are NOT accurate to the real cameras TODO: NOOOO TEHY ARE USED FOR GTSAM WHY MICHAELLL
  public static final CameraIntrinsics[] CAMERA_INTRINSICS = {
      new CameraIntrinsics(  // FRONT_LEFT_2025 CHECKED 7/29/2025
          1600, 1200,
          1286.9714942340822, 1286.958397164024, 801.3013669030433, 643.6657753564633,
          new double[] { 0.03330603946791099,-0.028149602298339575,-3.2480645802923945E-4,-9.9467580835956E-5,-0.01790312270432083,-5.6026682560497186E-5,0.004799695107552726,0.002517423938318154 }),
      new CameraIntrinsics( // BACK_LEFT_OFFSEASON CHECKED 7/29/2025
          1280, 800,
          739.5822727634005, 739.7612163752874, 677.842550740673, 412.60902494768686,
          new double[] { -0.11247457404784804,-0.08146986467182629,-4.532025251250321E-4,-4.5386987869399753E-4,0.32286142215868635,-0.23912601671263523,0.14071459566886046,0.24341038492753928 }),
      new CameraIntrinsics( // BACK_RIGHT_OFFSEASON CHECKED 7/29/2025
          1280, 800,
          903.6362625357887, 904.0894526094557, 659.272169009578, 419.33183667235414,
          new double[] { 0.04817038713247634,-0.06653199679614717,2.440943204330648E-4,-1.1668007422068571E-4,0.010742825203920284,-0.00217144171086648,0.0051145983132926676,0.0012068741948585884 })
  };

  // cad guesstimate
  public static final Rectangle2d FIELD = new Rectangle2d(
    new Translation2d(0, 0),
    new Translation2d(17.558, 8.065)
  );
}
