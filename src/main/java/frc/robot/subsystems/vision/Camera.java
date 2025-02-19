package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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
  private final double coefficientFactor = 10_000.0;

  // kinda ugly ik ik
  private Pose2d lastRobotPose;

  private final AprilTagFieldLayout field;

  public Camera(AprilTagFieldLayout field, String name, Transform3d transform, CameraIntrinsics intrinsics) {
    this.name = name;
    this.transform = transform;
    this.intrinsics = intrinsics;
    this.field = field;
    path = "Vision/" + name.replace("_", "");

    camera = new PhotonCamera(name);

    var strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    var fallbackStrategy = PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT;
    estimator = new PhotonPoseEstimator(this.field, strategy, transform);
    estimator.setMultiTagFallbackStrategy(fallbackStrategy);
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
    // TODO: readd
    // if (!FIELD.contains(estPose.toPose2d())) {
    // return Optional.empty();
    // }

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

  public void logCamTransform(Pose2d robotPose) {
    Pose3d camPose = new Pose3d(robotPose);
    camPose = camPose.transformBy(transform);
    Logger.recordOutput(path + "/CamTransform", camPose);
  }

  public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {

    lastRobotPose = poseEstimator.getEstimatedPosition();
    boolean visionWasMeasured = false;
    List<PhotonPipelineResult> pipelineResults = getPipelineResults();
    for (PhotonPipelineResult result : pipelineResults) {
      Optional<EstimatedRobotPose> poseResult = estimator.update(result);
      if (poseResult.isEmpty())
        continue;

      EstimatedRobotPose pose = poseResult.get();

      Logger.recordOutput(path + "/EstPoseUnfiltered", pose.estimatedPose);
      var filteredPose = filterPose(pose);
      if (filteredPose.isEmpty())
        continue;

      int numTargets = pose.targetsUsed.size();
      double avgTagArea = 0;
      for (var target : result.targets) {
        avgTagArea += target.area;
      }
      avgTagArea /= numTargets;

      double xyStdDev = xyStdDevCoefficient * Math.pow(avgTagArea, 2.0) / numTargets * coefficientFactor;
      double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgTagArea, 2.0) / numTargets * coefficientFactor;

      if (numTargets <= 1)
        thetaStdDev = Double.POSITIVE_INFINITY;

      Logger.recordOutput(path + "/XyStdDev", xyStdDev);
      Logger.recordOutput(path + "/ThetaStdDev", thetaStdDev);
      Logger.recordOutput(path + "/NumTargets", numTargets);
      Logger.recordOutput(path + "/AvgTagArea", avgTagArea);
      Logger.recordOutput(path + "/EstPose", pose.estimatedPose);

      var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, thetaStdDev);

      poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), result.getTimestampSeconds(), stddevs);
      visionWasMeasured = true;
    }

    if (!visionWasMeasured) {
      Logger.recordOutput(path + "/XyStdDev", -1d);
      Logger.recordOutput(path + "/ThetaStdDev", -1d);
      Logger.recordOutput(path + "/NumTargets", 0);
      Logger.recordOutput(path + "/AvgTagArea", -1d);
      Logger.recordOutput(path + "/EstPose", new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
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
}
