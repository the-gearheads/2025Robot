package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraIntrinsics;

public class VisionConstants {

  public static final String[] CAMERA_NAMES = { "FRONT_LEFT_2025",  "BACK_LEFT_2025", "BACK_RIGHT"};

  public static final Transform3d[] CAMERA_TRANSFORMS = {
      new Transform3d(  // FRONT_LEFT_2025
          new Translation3d(0.36322, 0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),
      new Transform3d( // BACK_LEFT_2025
          new Translation3d(-0.3683, 0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),
      new Transform3d( // BACK_RIGHT
          new Translation3d(-0.3683, -0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))

  };

  public static final CameraIntrinsics[] CAMERA_INTRINSICS = {
      new CameraIntrinsics(  // FRONT_LEFT_2025
          1600, 1200,
          1286.9714942340822, 801.3013669030433, 662.3371068271363, 643.6657753564633,
          new double[] { 0.03330603946791099,-0.028149602298339575,-3.2480645802923945E-4,-9.9467580835956E-5,-0.01790312270432083,-5.6026682560497186E-5,0.004799695107552726,0.002517423938318154 }),
      new CameraIntrinsics( // BACK LEFT
          1280, 800,
          907.5481543996266, 907.2864231011899, 615.788934769545, 384.31897050631306,
          new double[] { 0.04546920576986492,-0.060969029351137745,5.994746339424619E-5,-5.844773226102312E-4,0.008910675549702082,-0.0021265005451531174,0.00603543079528509,0.0019224441649826043 }),
      new CameraIntrinsics( // BACK RIGHT
          1280, 800,
          903.6362625357887, 904.0894526094557, 659.272169009578, 419.33183667235414,
          new double[] { 0.04817038713247634,-0.06653199679614717,2.440943204330648E-4,-1.1668007422068571E-4,0.010742825203920284,-0.00217144171086648,0.0051145983132926676,0.0012068741948585884 })
  };
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.POSITIVE_INFINITY);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
}
