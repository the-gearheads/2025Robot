package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraIntrinsics;

public class VisionConstants {

  public static final String[] CAMERA_NAMES = { "FRONT_LEFT_2025",  "BACK_LEFT_2025", "BACK_RIGHT"};

  public static final Transform3d[] CAMERA_TRANSFORMS = {
      new Transform3d(  // FRONT_LEFT_2025
          new Translation3d(0.36322, 0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(15))),
      new Transform3d( // BACK_LEFT_2025
          new Translation3d(-0.3683, 0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(0))),
      new Transform3d( // BACK_RIGHT
          new Translation3d(-0.3683, -0.1778, 0.16891),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-15)))

  };

  public static final CameraIntrinsics[] CAMERA_INTRINSICS = {
      new CameraIntrinsics(  // FRONT_LEFT_2025
          1600, 1200,
          1286.9714942340822, 801.3013669030433, 662.3371068271363, 643.6657753564633,
          new double[] { 0.03330603946791099,-0.028149602298339575,-3.2480645802923945E-4,-9.9467580835956E-5,-0.01790312270432083,-5.6026682560497186E-5,0.004799695107552726,0.002517423938318154 }),
      new CameraIntrinsics( // BACK LEFT
          1280, 800,
          737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
          new double[] { 0.15288116557227518, -0.2878953642242236, -0.0010986978034486703, 0.0011333394853758716,
              0.12276685039910991 }),
      new CameraIntrinsics(  // BACK_LEFT_2025
          1280, 800,
          737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
          new double[] { 0.15288116557227518, -0.2878953642242236, -0.0010986978034486703, 0.0011333394853758716,
              0.12276685039910991 }),
      new CameraIntrinsics(  // BACK_RIGHT
          1280, 800,
          737.6136442454854, 733.1927575565593, 662.3371068271363, 435.9984845786,
          new double[] { 0.15288116557227518, -0.2878953642242236, -0.0010986978034486703, 0.0011333394853758716,
              0.12276685039910991 })
  };
}
