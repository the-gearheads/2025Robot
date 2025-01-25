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

  public static final String[] CAMERA_NAMES = { "FRONT_RIGHT", "FRONT_LEFT", "BACK_LEFT" };
  public static final Transform3d[] CAMERA_TRANSFORMS = {
      new Transform3d(
          new Translation3d(0.3509899, -0.15261844, 0.21686012),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-25.5), Units.degreesToRadians(29.0))),
      new Transform3d( // FRONT LEFT
          new Translation3d(0.3509899, 0.1526159, 0.2167636),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21), Units.degreesToRadians(-31.0))),
      new Transform3d( // BACK LEFT
          new Translation3d(-0.36195, 0.265049, 0.231775),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21.9), Units.degreesToRadians(180)))

  };

  public static final CameraIntrinsics[] CAMERA_INTRINSICS = {
      new CameraIntrinsics(
          1280, 800,
          737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
          new double[] { 0.15288116557227518, -0.2878953642242236, -0.0010986978034486703, 0.0011333394853758716,
              0.12276685039910991 }),
      new CameraIntrinsics(
          1280, 800,
          737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
          new double[] { 0.15288116557227518, -0.2878953642242236, -0.0010986978034486703, 0.0011333394853758716,
              0.12276685039910991 }),
      new CameraIntrinsics(
          1280, 800,
          737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
          new double[] { 0.15288116557227518, -0.2878953642242236, -0.0010986978034486703, 0.0011333394853758716,
              0.12276685039910991 })
  };
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.POSITIVE_INFINITY);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
}
