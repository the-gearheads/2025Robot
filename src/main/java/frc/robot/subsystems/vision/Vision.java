// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.constants.VisionConstants.*;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout field;
  private VisionSim sim;
  private Swerve swerve;

  private final Camera frontLeft, frontRight, backLeft;

  public Vision(Swerve swerve) {
    this.swerve = swerve;
    // might want to remove this before comp
    if(Robot.isSimulation())
      PhotonCamera.setVersionCheckEnabled(false);  

    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("ERROR Opening apriltag field layout file");
      System.out.println(AprilTagFields.k2024Crescendo.m_resourceFile);
    }

    frontLeft = new Camera(field, FRONT_LEFT_NAME, FRONT_LEFT_TRANSFORM, FRONT_LEFT_INTRINSICS);
    frontRight = new Camera(field, FRONT_RIGHT_NAME, FRONT_RIGHT_TRANSFORM, FRONT_RIGHT_INTRINSICS);
    backLeft = new Camera(field, BACK_LEFT_NAME, BACK_LEFT_TRANSFORM, BACK_LEFT_INTRINSICS);

    sim = new VisionSim();
    sim.addCamera(frontLeft);
    sim.addCamera(frontRight);
    sim.addCamera(backLeft);
  }

  /**
   * Updates the passed pose estimator with vision estimates.
   * @param poseEstimator
   * @return Whether a vision measurement was added
   */
  public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
    boolean tfl = frontLeft.feedPoseEstimator(poseEstimator);
    boolean tfr = frontRight.feedPoseEstimator(poseEstimator);
    boolean tbl = backLeft.feedPoseEstimator(poseEstimator);

    return tfl || tfr || tbl;
  }

  @Override
  public void periodic() {
    // sim.periodic(swerve.getPoseWheelsOnly());
    frontLeft.logCamTransform(swerve.getPose());
    frontRight.logCamTransform(swerve.getPose());
    backLeft.logCamTransform(swerve.getPose());
  }
}
