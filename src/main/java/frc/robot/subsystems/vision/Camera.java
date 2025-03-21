package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.MiscConstants.AUTO_ALIGN_DIST_THRESHOLD;
import static frc.robot.constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.ReefPositions;

public class Camera {

  public final String name;
  public final String path;
  public final Transform3d transform;
  public final CameraIntrinsics intrinsics;

  public final PhotonCamera camera;
  public final PhotonPoseEstimator estimator;

  private final double MAX_PITCHROLL = Units.degreesToRadians(5);
  private final double MAX_Z = Units.inchesToMeters(7);

  private final double xyStdDevCoefficient = 0.08;
  private final double thetaStdDevCoefficient = 0.16;
  private final double coefficientFactor = 0.7;

  // kinda ugly ik ik
  private Pose2d lastRobotPose;
  private boolean disabled;

  private final AprilTagFieldLayout field;

  private int tagFilteringTag = -1;
  Supplier<Pose2d> robotPoseSupplier;
  DoubleSupplier fusedHeadingSupplier;
  DoubleSupplier gyroAngleSupplier;
  Rotation2d gyroOffset = new Rotation2d();

  LoggedNetworkNumber headingScaleFactor = new LoggedNetworkNumber("Vision/HeadingScaleFactor", CONSTRAINED_PNP_HEADING_SCALE_FACTOR);

  public Camera(AprilTagFieldLayout field, String name, Transform3d transform, CameraIntrinsics intrinsics, DoubleSupplier fusedHeadingSupplier, DoubleSupplier gyroAngleSupplier, Supplier<Pose2d> robotPoseSupplier, PoseStrategy strategy) {
    this.name = name;
    this.transform = transform;
    this.intrinsics = intrinsics;
    this.field = field;
    this.robotPoseSupplier = robotPoseSupplier;
    path = "Vision/" + name.replace("_", "");
    OpenCVHelp.forceLoadOpenCV();

    camera = new PhotonCamera(name);
    var fallbackStrategy = PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT;
    estimator = new PhotonPoseEstimator(this.field, strategy, transform);
    estimator.setMultiTagFallbackStrategy(fallbackStrategy);
    this.fusedHeadingSupplier = fusedHeadingSupplier;
    this.gyroAngleSupplier = gyroAngleSupplier;

  }

  public List<PhotonPipelineResult> getPipelineResults() {
    return camera.getAllUnreadResults();
  }

  private Optional<Pose3d> filterPose(EstimatedRobotPose estimatedPose) {
    Pose3d estPose = estimatedPose.estimatedPose;
    double pitch = estPose.getRotation().getX();
    double roll = estPose.getRotation().getY();
    if (Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL
        || Math.abs(estPose.getTranslation().getZ()) > MAX_Z) {
      return Optional.empty();
    }

    if (!FIELD.contains(estPose.toPose2d().getTranslation())) {
      return Optional.empty();
    }

    // advantagekit viz stuff
    ArrayList<Pose3d> allTagPoses = new ArrayList<>();
    var currentPose3d = new Pose3d(lastRobotPose);
    for (var detectionEntry : estimatedPose.targetsUsed) {
      var detection = detectionEntry.getBestCameraToTarget();
      var fieldToTag = currentPose3d.transformBy(transform).transformBy(detection);
      allTagPoses.add(fieldToTag);
    }
    Logger.recordOutput(path + "/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    return Optional.of(estPose);
  }

  public void filterByTagId(int targetID)  {
    tagFilteringTag = targetID;
  }

  public void disableTagIdFiltering() {
    tagFilteringTag = -1;
  }

  public void logCamTransform(Pose2d robotPose) {
    Pose3d camPose = new Pose3d(robotPose);
    camPose = camPose.transformBy(transform);
    Logger.recordOutput(path + "/CamTransform", camPose);
  }

  public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator, Rotation2d gyroOffset) {
    Logger.recordOutput(path + "/isDisabled", isDisabled());
    if(disabled) {
      return false;
    }
    Logger.recordOutput(path + "/PoseStrategy", estimator.getPrimaryStrategy());
    lastRobotPose = poseEstimator.getEstimatedPosition();
    boolean visionWasMeasured = false;
    List<PhotonPipelineResult> pipelineResults = getPipelineResults();
    Optional<EstimatedRobotPose> poseResult;
    for (PhotonPipelineResult result : pipelineResults) {
      if(estimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE || estimator.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP) {
        boolean headingFree = DriverStation.isDisabled();
        var constrainedPnpParams = new PhotonPoseEstimator.ConstrainedSolvepnpParams(headingFree, headingScaleFactor.get());
        Rotation2d gyroAngle = Rotation2d.fromRadians(gyroAngleSupplier.getAsDouble());
        estimator.addHeadingData(Timer.getFPGATimestamp(), gyroAngle.plus(gyroOffset));
        Logger.recordOutput("Vision/GyroAnglePlusOffset", gyroAngle.plus(gyroOffset));
        if (estimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE && tagFilteringTag != -1) {
          if (result.hasTargets()) {
            if (result.getBestTarget().getFiducialId() != tagFilteringTag)
              continue;
          }
        }
        poseResult = estimator.update(result, camera.getCameraMatrix(), camera.getDistCoeffs(), Optional.of(constrainedPnpParams));
      }
      else {
        poseResult = estimator.update(result);
      }
      if (poseResult.isEmpty())
        continue;

      EstimatedRobotPose pose = poseResult.get();

      // Logger.recordOutput(path + "/Result", result);
      Logger.recordOutput(path + "/EstPoseUnfiltered", pose.estimatedPose);
      Optional<Pose3d> filteredPose = filterPose(pose);
      if (filteredPose.isEmpty())
        continue;

      int numTargets = pose.targetsUsed.size();
      double avgTagDist = 0;
      for (var target : result.targets) {
        avgTagDist += target.bestCameraToTarget.getTranslation().getNorm();
      }
      avgTagDist /= numTargets;

      double xyStdDev = xyStdDevCoefficient * Math.pow(avgTagDist, 2) / (numTargets * coefficientFactor);
      double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgTagDist, 2) / (numTargets * coefficientFactor);

      // if (numTargets <= 1)
      //   thetaStdDev = Double.POSITIVE_INFINITY;
      if (estimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
        xyStdDev /= 15;
        thetaStdDev /= 15;

        double distanceToReefTag = filteredPose.get().toPose2d().getTranslation().getDistance(ReefPositions.getClosestReefTagPose(robotPoseSupplier.get()).getTranslation());
        Logger.recordOutput(path + "/ReefTagDist", distanceToReefTag);
        if (distanceToReefTag > AUTO_ALIGN_DIST_THRESHOLD + 0.75)
          continue;
      }

      Logger.recordOutput(path + "/XyStdDev", xyStdDev);
      Logger.recordOutput(path + "/ThetaStdDev", thetaStdDev);
      Logger.recordOutput(path + "/NumTargets", numTargets);
      Logger.recordOutput(path + "/AvgTagDist", avgTagDist);
      Logger.recordOutput(path + "/EstPose", pose.estimatedPose);

      // var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, thetaStdDev);
      var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0, 0, 0);

      poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), result.getTimestampSeconds(), stddevs);
      visionWasMeasured = true;
    }

    if (!visionWasMeasured) {
      Logger.recordOutput(path + "/XyStdDev", -1d);
      Logger.recordOutput(path + "/ThetaStdDev", -1d);
      Logger.recordOutput(path + "/NumTargets", 0);
      Logger.recordOutput(path + "/AvgTagDist", -1d);
      Logger.recordOutput(path + "/EstPose", new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
      Logger.recordOutput(path + "/EstPoseUnfiltered", new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
      Logger.recordOutput(path + "/TagPoses", new Pose3d[0]);
      return false;
    }

    return true;
  }

  public SimCameraProperties getSimProperties() {
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(intrinsics.resX, intrinsics.resY, intrinsics.getCameraMatrix(),
        intrinsics.getDistCoeffs());

    // Approximate detection noise with average and standard deviation error in
    // pixels.
    properties.setCalibError(0.02, 0.05);
    // Set the camera image capture framerate (Note: this is limited by robot loop
    // rate).
    properties.setFPS(30);
    // The average and standard deviation in milliseconds of image data latency.
    properties.setAvgLatencyMs(35);
    properties.setLatencyStdDevMs(7);

    return properties;
  }

  public void setPoseStrategy(PoseStrategy strategy) {
    estimator.setPrimaryStrategy(strategy);
  }

  public void disable() {
    disabled = true;
  }

  public void enable() {
    disabled = false;
  }

  public boolean isDisabled() {
    return disabled;
  } 
}
