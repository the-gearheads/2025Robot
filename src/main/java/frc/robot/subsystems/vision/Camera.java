package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

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
  private String path;
  private Transform3d transform;
  private CameraIntrinsics intrinsics;
  private AprilTagFieldLayout field;
  private PhotonCamera camera;
  
  public final PhotonPoseEstimator estimator;
  
  private final double MAX_PITCHROLL_DEGREES = 5;
  private final double MAX_PITCHROLL = Units.degreesToRadians(MAX_PITCHROLL_DEGREES);
  private final double MAX_Z_INCHES = 7;
  private final double MAX_Z = Units.inchesToMeters(MAX_Z_INCHES);

  private final double XY_STD_DEV_COEFFICIENT = 0.08;
  private final double THETA_STD_DEV_COEFFICIENT = 0.16;
  private final double COEFFICIENT_FACTOR = 10_000.0;

  private final String PATH_VISION = "Vision/";
  private final String PATH_TAG_POSES = "/TagPoses";
  private final String PATH_CAM_TRANSFORM = "/CamTransform";
  private final String PATH_EST_POSE_UNFILTERED = "/EstPoseUnfiltered";
  private final String PATH_XY_STD_DEV = "/XyStdDev";
  private final String PATH_THETA_STD_DEV = "/ThetaStdDev";
  private final String PATH_NUM_TARGETS = "/NumTargets";
  private final String PATH_AVG_TAG_AREA = "/AvgTagArea";
  private final String PATH_EST_POSE = "/EstPose";

  // kinda ugly ik ik
  private Pose2d lastRobotPose;

  public Camera(AprilTagFieldLayout field, PhotonCamera camera, Transform3d transform, CameraIntrinsics intrinsics) {
    this.transform = transform;
    this.intrinsics = intrinsics;
    this.field = field;
    this.path = PATH_VISION + camera.getName().replace("_", "");

    estimator = new PhotonPoseEstimator(this.field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transform);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
  }

  public PhotonCamera getCamera() {
    if (Objects.isNull(camera)) 
      return camera;
    return new PhotonCamera("");
  }

  public Transform3d getTransform() {
    return transform;
  }

  public List<PhotonPipelineResult> getPipelineResults() {
    return camera.getAllUnreadResults();
  }

  private Optional<Pose3d> filterPose(EstimatedRobotPose estimatedPose) {
    Pose3d estPose = estimatedPose.estimatedPose;
    double pitch = estPose.getRotation().getX();
    double roll = estPose.getRotation().getY();
    if (Math.abs(pitch) > MAX_PITCHROLL 
      || Math.abs(roll) > MAX_PITCHROLL
      || Math.abs(estPose.getTranslation().getZ()) > MAX_Z) {
      return Optional.empty();
    }
    // TODO: readd
    // if (!FIELD.contains(estPose.toPose2d())) {
    // return Optional.empty();
    // }

    // advantagekit viz stuff
    ArrayList<Pose3d> allTagPoses = new ArrayList<>();
    Pose3d currentPose3d = new Pose3d(lastRobotPose);
    Transform3d detection;
    Pose3d fieldToTag;
    for (var detectionEntry : estimatedPose.targetsUsed) {
      detection = detectionEntry.getBestCameraToTarget();
      fieldToTag = currentPose3d.transformBy(transform).transformBy(detection);
      allTagPoses.add(fieldToTag);
    }
    Logger.recordOutput(path + PATH_TAG_POSES, allTagPoses.toArray(Pose3d[]::new));

    return Optional.of(estPose); // TODO: ADD ERROR CATCHING OR SOMETHING ELSE TO REMOVE THE OPTIONAL.OF / ASK MORE ABOUT THIS
  }

  public void logCamTransform(Pose2d robotPose) {
    Pose3d camPose = new Pose3d(robotPose);
    camPose = camPose.transformBy(transform);
    Logger.recordOutput(path + PATH_CAM_TRANSFORM, camPose);
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

      Logger.recordOutput(path + PATH_EST_POSE_UNFILTERED, pose.estimatedPose);
      var filteredPose = filterPose(pose);
      if (filteredPose.isEmpty())
        continue;

      int numTargets = pose.targetsUsed.size();
      double avgTagArea = 0;
      for (var target : result.targets) {
        avgTagArea += target.area;
      }
      avgTagArea /= numTargets;

      double xyStdDev = XY_STD_DEV_COEFFICIENT * Math.pow(avgTagArea, 2.0) / numTargets * COEFFICIENT_FACTOR;
      double thetaStdDev = THETA_STD_DEV_COEFFICIENT * Math.pow(avgTagArea, 2.0) / numTargets * COEFFICIENT_FACTOR;

      if (numTargets <= 1)
        thetaStdDev = Double.POSITIVE_INFINITY;

      Logger.recordOutput(path + PATH_XY_STD_DEV, xyStdDev);
      Logger.recordOutput(path + PATH_THETA_STD_DEV, thetaStdDev);
      Logger.recordOutput(path + PATH_NUM_TARGETS, numTargets);
      Logger.recordOutput(path + PATH_AVG_TAG_AREA, avgTagArea);
      Logger.recordOutput(path + PATH_EST_POSE, pose.estimatedPose);

      var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, thetaStdDev);

      poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), result.getTimestampSeconds(), stddevs);
      visionWasMeasured = true;
    }

    if (!visionWasMeasured) {
      Logger.recordOutput(path + PATH_XY_STD_DEV, -1d);
      Logger.recordOutput(path + PATH_THETA_STD_DEV, -1d);
      Logger.recordOutput(path + PATH_NUM_TARGETS, 0);
      Logger.recordOutput(path + PATH_AVG_TAG_AREA, -1d);
      Logger.recordOutput(path + PATH_EST_POSE, new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
      Logger.recordOutput(path + PATH_TAG_POSES, new Pose3d[0]);
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
