// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.constants.VisionConstants.*;

public class Vision extends SubsystemBase {
  public static AprilTagFieldLayout field;
  private VisionSim sim = new VisionSim();
  private Swerve swerve;
  private Rotation2d gyroOffset;

  private Camera[] cameras = new Camera[CAMERA_NAMES.length];

  public Vision(Swerve swerve) {
    this.swerve = swerve;
    // might want to remove this before comp
    if(Robot.isSimulation())
      PhotonCamera.setVersionCheckEnabled(false);  

    String layoutPath = "";
    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      // layoutPath = new File(Filesystem.getDeployDirectory(), "wpicalFieldMap.json").getAbsolutePath();
      // field = new AprilTagFieldLayout(layoutPath);
    } catch (IOException e) {
      System.out.println("ERROR Opening apriltag field layout file");
      System.out.println(layoutPath);
    }

  
    for (int i = 0; i<CAMERA_NAMES.length; i++) {
      cameras[i] = new Camera(field, CAMERA_NAMES[i], CAMERA_TRANSFORMS[i], CAMERA_INTRINSICS[i], ()->swerve.getPose().getRotation().getRadians(), ()->swerve.getGyroRotation().getRadians());
      sim.addCamera(cameras[i]);
    }

  }

  /**
   * Updates the passed pose estimator with vision estimates.
   * @param poseEstimator
   * @return Whether a vision measurement was added
   */
  public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
    boolean posed = false;
    if(DriverStation.isDisabled()) {
      gyroOffset = swerve.getPose().getRotation().minus(swerve.getGyroRotation());
    }
    for (Camera camera : cameras) {
      posed |= camera.feedPoseEstimator(poseEstimator, gyroOffset);
    }
    
    return posed;
  }

  @Override
  public void periodic() {
    sim.periodic(swerve.getPoseWheelsOnly());
    for (Camera camera : cameras) {
      camera.logCamTransform(swerve.getPose());
    }
  }
}
