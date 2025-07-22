// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Camera.VisionObservation;
import frc.robot.subsystems.vision.gtsam.GtsamInterface;

import static frc.robot.constants.VisionConstants.*;

public class Vision extends SubsystemBase {
  public static AprilTagFieldLayout field;
  private VisionSim sim = new VisionSim();
  private Swerve swerve;
  private Rotation2d gyroOffset;

  private Camera[] cameras = new Camera[CAMERA_NAMES.length];
  private GtsamInterface gtsam = new GtsamInterface(List.of(CAMERA_NAMES));

  LoggedNetworkBoolean useGtsam = new LoggedNetworkBoolean("AdvantageKit/RealOutputs/Vision/UseGtsam", Robot.isReal() ? USE_GTSAM_DEFAULT : false);
  Optional<Pose3d> firstVisionEstimate = Optional.empty(); 

  @AutoLogOutput
  private int cameraPriority = -1;

  public Vision(Swerve swerve) {
    this.swerve = swerve;
    // might want to remove this before comp
    if(Robot.isSimulation())
      PhotonCamera.setVersionCheckEnabled(false);

    String layoutPath = "";
    try {
      field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      // layoutPath = new File(Filesystem.getDeployDirectory(), "REEFSCAPE FIELD MAP WITHOUT BARGE TAGS.json").getAbsolutePath();
      // field = new AprilTagFieldLayout(layoutPath);
    } catch (IOException e) {
      System.out.println("ERROR Opening apriltag field layout file");
      System.out.println(layoutPath);
    }


    for (int i = 0; i<CAMERA_NAMES.length; i++) {
      cameras[i] = new Camera(field, CAMERA_NAMES[i], CAMERA_TRANSFORMS[i], SIM_CAMERA_INTRINSICS[i], ()->swerve.getPoseMultitag().getRotation().getRadians(), ()->swerve.getPoseWheelsOnly().getRotation().getRadians(), swerve::getPose, INITAL_CAMERA_STRATEGIES[i]);
      sim.addCamera(cameras[i]);
    }

  }

  private void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator, Camera cam, VisionObservation observation) {
    if (firstVisionEstimate.isEmpty()) {
      firstVisionEstimate = Optional.of(observation.poseResult().estimatedPose);
      Logger.recordOutput("Vision/FirstVisionEstimate", firstVisionEstimate.get());
    }
    poseEstimator.addVisionMeasurement(observation.poseResult().estimatedPose.toPose2d(), observation.poseResult().timestampSeconds, observation.stddevs());
    if(usingGtsam()) {
      gtsam.sendVisionUpdate(cam.name, observation.poseResult()); // -really- we're just using the corners but this is what we got
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
      gyroOffset = swerve.getPose().getRotation().minus(swerve.getPoseWheelsOnly().getRotation());
    }
    Logger.recordOutput("Vision/GyroOffset", gyroOffset);
    if (cameraPriority != -1) {
      // Prioritize observations from a prioritized camera
      for (VisionObservation observation : cameras[cameraPriority].getObservations(poseEstimator, gyroOffset)) {
        addVisionMeasurement(poseEstimator, cameras[cameraPriority], observation);
        posed = true;
      }
      // But if there's nothing, try the others
      if (!posed) {
        for (int i=0; i<cameras.length; i++) {
          if(i != cameraPriority)
          for (VisionObservation observation : cameras[i].getObservations(poseEstimator, gyroOffset)) {
            addVisionMeasurement(poseEstimator, cameras[i], observation);
            posed = true;
          }
        }
      }
    } else {
      // Otherwise just do em all
      for (Camera camera : cameras) {
        for (VisionObservation observation : camera.getObservations(poseEstimator, gyroOffset)) {
          addVisionMeasurement(poseEstimator, camera, observation);
          posed = true;
        }
      }
    }
    // $$ \ce{H2O <-> H2O} $$
    return posed;
  }

  Pose2d lastOdom = new Pose2d();
  @Override
  public void periodic() {
    sim.periodic(swerve.getPoseWheelsOnly());
    if(usingGtsam()) {
      Pose2d odom = swerve.getPoseWheelsOnly();
      Twist2d odomTwist2d = lastOdom.log(odom);
      lastOdom = odom;
      // Twist3d odomTwist3d = new Pose3d().log(new Pose3d(new Pose2d().exp(odomTwist2d)));
      Twist3d odomTwist3d = new Twist3d(odomTwist2d.dx, odomTwist2d.dy, 0, 0, 0, odomTwist2d.dtheta);
      if (firstVisionEstimate.isPresent()) {
        gtsam.sendOdomUpdate(WPIUtilJNI.now(), odomTwist3d, firstVisionEstimate.get());
      }
      Logger.recordOutput("Vision/Gtsam/PoseEstimate", gtsam.getLatencyCompensatedPoseEstimate());
      Logger.recordOutput("Vision/Gtsam/RawPoseEstimate", gtsam.getRawPoseEstimate());
      Logger.recordOutput("Vision/Gtsam/LoopTimeMs", gtsam.getLoopTimeMs());
      Logger.recordOutput("Vision/Gtsam/ReadyToOptimize", gtsam.isReadyToOptimize());
      Logger.recordOutput("Vision/Gtsam/HadIssue", gtsam.hadIssue());
    }
    for (Camera camera : cameras) {
      camera.logCamTransform(swerve.getPose());
      if(usingGtsam()) {
        gtsam.setCamIntrinsics(camera.name, camera.camera.getCameraMatrix(), camera.camera.getDistCoeffs());
        gtsam.sendRobotToCam(camera.name, camera.transform);
      }
    }
  }

  public Pose3d getPoseGtsam() {
    return gtsam.getLatencyCompensatedPoseEstimate();
  }

  public void setPoseStrategy(int cameraIndex, PoseStrategy strategy) {
    cameras[cameraIndex].setPoseStrategy(strategy);
  }

  public void defaultPoseStrategies() {
    for (int i=0; i<cameras.length; i++) {
      cameras[i].setPoseStrategy(INITAL_CAMERA_STRATEGIES[i]);
    }
  }

  public void disableCamera(int cameraIndex) {
    cameras[cameraIndex].disable();
  }

  public void disable() {
    for (int i=0; i<cameras.length; i++) {
      cameras[i].disable();
    }
  }

  public void enable() {
    for (int i=0; i<cameras.length; i++) {
      cameras[i].enable();
    }
  }

  public void enableCamera(int cameraIndex) {
    cameras[cameraIndex].enable();
  }

  public void filterTagById(int cameraIndex, int tagID) {
    cameras[cameraIndex].filterByTagId(tagID);
  }

  public void disableIdFiltering(int cameraIndex) {
    cameras[cameraIndex].disableTagIdFiltering();
  }

  public void setCameraPreference(int cameraIndex) {
    cameraPriority = cameraIndex;
  }

  public void disableCameraPreference() {
    cameraPriority = -1;
  }

  public void setGyroOffset(Rotation2d offset) {
    gyroOffset = offset;
  }

  public boolean usingGtsam() {
    return useGtsam.get();
  }

}
