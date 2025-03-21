package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
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

public class Camera {
  private static final int LATENCY_STDDEV_MS = 7;
  public final String name;
  public final String path;
  public final Transform3d transform;
  public final CameraIntrinsics intrinsics;

  public final PhotonCamera camera;
  public final PhotonPoseEstimator estimator;

  private final double MAX_PITCHROLL = Units.degreesToRadians(5);
  private final double MAX_Z = Units.inchesToMeters(7);

  private final double XY_STDDEV_COEFFICIENT = 0.08;
  private final double THETA_STDDEV_COEFFICIENT = 0.16;
  private final double COEFFICIENT_FACTOR = 0.7;
  private static final double CALIB_ERROR_AVG = 0.02;
  private static final double CALIB_ERROR_STDDEV = 0.05;
  private static final double FPS = 30;
  private static final double AVG_LATENCY_MS = 35;

  // kinda ugly ik ik
  private Pose2d lastRobotPose;
  private boolean disabled;

  private final AprilTagFieldLayout field;

  private int tagFilteringTag = -1;
  DoubleSupplier fusedHeadingSupplier;
  DoubleSupplier gyroAngleSupplier;
  Rotation2d gyroOffset = new Rotation2d();

  LoggedNetworkNumber headingScaleFactor = new LoggedNetworkNumber("Vision/HeadingScaleFactor",
      CONSTRAINED_PNP_HEADING_SCALE_FACTOR);

  public Camera(AprilTagFieldLayout field, String name, Transform3d transform, CameraIntrinsics intrinsics,
      DoubleSupplier fusedHeadingSupplier, DoubleSupplier gyroAngleSupplier, PoseStrategy strategy) {
    this.name = name;
    this.transform = transform;
    this.intrinsics = intrinsics;
    this.field = field;
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
    if (Objects.isNull(estimatedPose)) {
      return Optional.empty();
    }

    Pose3d estPose = estimatedPose.estimatedPose;
    double pitch = estPose.getRotation().getX();
    double roll = estPose.getRotation().getY();

    if (isOutOfBounds(pitch, MAX_PITCHROLL)
        || isOutOfBounds(roll, MAX_PITCHROLL)
        || isOutOfBounds(estPose.getTranslation().getZ(), MAX_Z)
        || !FIELD.contains(estPose.toPose2d().getTranslation())) {
      return Optional.empty();
    }

    setAdvantageKitVision(estimatedPose);

    return Optional.of(estPose);
  }

  private void setAdvantageKitVision(EstimatedRobotPose estimatedPose) {
    // advantagekit viz stuff
    ArrayList<Pose3d> allTagPoses = new ArrayList<>();
    var currentPose3d = new Pose3d(lastRobotPose);
    for (var detectionEntry : estimatedPose.targetsUsed) {
      var detection = detectionEntry.getBestCameraToTarget();
      var fieldToTag = currentPose3d.transformBy(transform).transformBy(detection);
      allTagPoses.add(fieldToTag);
    }
    Logger.recordOutput(path + "/TagPoses", allTagPoses.toArray(Pose3d[]::new));
  }

  private boolean isOutOfBounds(double value, double upperBound) {
    return Math.abs(value) > upperBound;
  }

  public void filterByTagId(int targetID) {
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
    if (disabled) {
      return false;
    }
    Logger.recordOutput(path + "/PoseStrategy", estimator.getPrimaryStrategy());

    boolean visionWasMeasured = false;
    List<PhotonPipelineResult> pipelineResults = getPipelineResults();
    Optional<EstimatedRobotPose> poseResult;
    setLastRobotPose(poseEstimator);

    for (PhotonPipelineResult result : pipelineResults) {
      poseResult = calculatePose(result);
      EstimatedRobotPose pose = getPose(poseResult);
      var filteredPose = filterPose(pose);
      if (!filteredPose.isEmpty()) {
        visionWasMeasured = measureVision(poseEstimator, result, pose); // TODO: why are we using pose and not
                                                                        // filteredPose?
      }
    }

    if (!visionWasMeasured) {
      visionNotMeasuredLogs();
      return false;
    }

    return true;
  }

  private void visionNotMeasuredLogs() {
    Logger.recordOutput(path + "/XyStdDev", -1d);
    Logger.recordOutput(path + "/ThetaStdDev", -1d);
    Logger.recordOutput(path + "/NumTargets", 0);
    Logger.recordOutput(path + "/AvgTagDist", -1d);
    Logger.recordOutput(path + "/EstPose", new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
    Logger.recordOutput(path + "/EstPoseUnfiltered",
        new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
    Logger.recordOutput(path + "/TagPoses", new Pose3d[0]);
  }

  private boolean measureVision(SwerveDrivePoseEstimator poseEstimator, PhotonPipelineResult result,
      EstimatedRobotPose pose) {
    boolean visionWasMeasured;
    int numTargets = pose.targetsUsed.size();
    double avgTagDist = getAvgTagDist(pose, result, numTargets);
    double xyStdDev = calculateStdDev(XY_STDDEV_COEFFICIENT, numTargets, avgTagDist);
    double thetaStdDev = calculateStdDev(THETA_STDDEV_COEFFICIENT, numTargets, avgTagDist);
    // var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev,
    // thetaStdDev);
    var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0, 0, 0);

    Logger.recordOutput(path + "/XyStdDev", xyStdDev);
    Logger.recordOutput(path + "/ThetaStdDev", thetaStdDev);
    Logger.recordOutput(path + "/NumTargets", numTargets);
    Logger.recordOutput(path + "/AvgTagDist", avgTagDist);
    Logger.recordOutput(path + "/EstPose", pose.estimatedPose);

    poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), result.getTimestampSeconds(), stddevs);
    visionWasMeasured = true; // TODO: under what circumstances would this be false?
    return visionWasMeasured;
  }

  private EstimatedRobotPose getPose(Optional<EstimatedRobotPose> poseResult) {
    if (poseResult.isEmpty()) {
      return null;
    }

    EstimatedRobotPose pose = poseResult.get();
    Logger.recordOutput(path + "/EstPoseUnfiltered", pose.estimatedPose);
    return pose;
  }

  private void setLastRobotPose(SwerveDrivePoseEstimator poseEstimator) {
    lastRobotPose = poseEstimator.getEstimatedPosition();
  }

  private double getAvgTagDist(EstimatedRobotPose pose, PhotonPipelineResult result, int numTargets) {
    double avgTagDist = 0;
    for (var target : result.targets) {
      avgTagDist += target.bestCameraToTarget.getTranslation().getNorm();
    }
    avgTagDist /= numTargets;
    return avgTagDist;
  }

  private double calculateStdDev(double coefficient, int numTargets, double avgTagDist) {
    final double STDDEV_RATIO = 15;
    double stdDev = coefficient * Math.pow(avgTagDist, 2) / (numTargets * COEFFICIENT_FACTOR);
    if (isStrategySolvePnpTrigDistance()) {
      stdDev /= STDDEV_RATIO;
    }
    return stdDev;
  }

  private Optional<EstimatedRobotPose> calculatePose(PhotonPipelineResult result) {
    if (isStrategySolvePnpTrigDistance()
        && tagFilteringTag != -1
        && result.hasTargets()
        && result.getBestTarget().getFiducialId() != tagFilteringTag) {
      return Optional.empty();
    } else if (isStrategySolvePnp()) {
      return updateEstimatorForSolvePnp(result);
    } else {
      return estimator.update(result);
    }
  }

  private Optional<EstimatedRobotPose> updateEstimatorForSolvePnp(PhotonPipelineResult result) {
    ConstrainedSolvepnpParams constrainedPnpParams = new PhotonPoseEstimator.ConstrainedSolvepnpParams(
      DriverStation.isDisabled(),
      headingScaleFactor.get()
    );
    Rotation2d gyroAngle = Rotation2d.fromRadians(gyroAngleSupplier.getAsDouble());
    Rotation2d GyroAnglePlusOffset = gyroAngle.plus(gyroOffset);
    
    Logger.recordOutput("Vision/GyroAnglePlusOffset", GyroAnglePlusOffset);
    
    estimator.addHeadingData(Timer.getFPGATimestamp(), GyroAnglePlusOffset);
    
    return estimator.update(result, camera.getCameraMatrix(), camera.getDistCoeffs(),
        Optional.of(constrainedPnpParams));
  }

  private boolean isStrategySolvePnp() {
    return isStrategySolvePnpTrigDistance() || estimator.getPrimaryStrategy() == PoseStrategy.CONSTRAINED_SOLVEPNP;
  }

  private boolean isStrategySolvePnpTrigDistance() {
    return estimator.getPrimaryStrategy() == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;
  }

  public SimCameraProperties getSimProperties() {
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(intrinsics.resX, intrinsics.resY, intrinsics.getCameraMatrix(),
        intrinsics.getDistCoeffs());

    // Approximate detection noise with average and standard deviation error in
    // pixels.
    properties.setCalibError(CALIB_ERROR_AVG, CALIB_ERROR_STDDEV);
    // Set the camera image capture framerate (Note: this is limited by robot loop
    // rate).
    properties.setFPS(FPS);
    // The average and standard deviation in milliseconds of image data latency.
    properties.setAvgLatencyMs(AVG_LATENCY_MS);
    properties.setLatencyStdDevMs(LATENCY_STDDEV_MS);

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
