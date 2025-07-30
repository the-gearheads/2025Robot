package frc.robot.subsystems.vision.gtsam;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;

public class GtsamInterface {

    private static class CameraInterface {
        StructArrayPublisher<TagDetection> tagPub;
        DoubleArrayPublisher camIntrinsicsPublisher;
        StructPublisher<Transform3d> robotTcamPub;
        private String name;

        public CameraInterface(String name) {
            this.name = name;
            tagPub = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("/gtsam_meme/" + name + "/input/tags", TagDetection.struct)
                    .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
            robotTcamPub = NetworkTableInstance.getDefault()
                    .getStructTopic("/gtsam_meme/" + name + "/input/robotTcam", Transform3d.struct)
                    .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
            camIntrinsicsPublisher = NetworkTableInstance.getDefault()
                    .getDoubleArrayTopic("/gtsam_meme/" + name + "/input/cam_intrinsics")
                    .publish(PubSubOption.sendAll(false), PubSubOption.keepDuplicates(false));
        }
    }

    StructPublisher<Twist3d> odomPub;
    StructPublisher<Pose3d> guessPub;
    Map<String, CameraInterface> cameras = new HashMap<>();
    StructSubscriber<Pose3d> optimizedPoseSub;
    DoubleArraySubscriber poseStddevSub;
    StringPublisher fieldLayoutPub;
    // StructArraySubscriber<Pose3d> optimizedTrajSub;

    DoubleSubscriber loopTimeSub;
    BooleanSubscriber readyToOptimizeSub;
    BooleanSubscriber hadIssueSub;

    // Estimated odom-only location relative to robot boot. We assume zero slip here
    Pose3d localOdometryPose = new Pose3d();
    TimeInterpolatableBuffer<Pose3d> odometryBuffer = TimeInterpolatableBuffer.createBuffer(5);

    public GtsamInterface(List<String> cameraNames) {
        odomPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/odom_twist", Twist3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        guessPub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/input/pose_initial_guess", Pose3d.struct)
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        optimizedPoseSub = NetworkTableInstance.getDefault()
                .getStructTopic("/gtsam_meme/output/optimized_pose", Pose3d.struct)
                .subscribe(null, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        poseStddevSub = NetworkTableInstance.getDefault()
                .getDoubleArrayTopic("/gtsam_meme/output/pose_stddev")
                .subscribe(null, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        // optimizedTrajSub = NetworkTableInstance.getDefault()
        //         .getStructArrayTopic("/gtsam_meme/output/optimized_traj", Pose3d.struct)
        //         .subscribe(null, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

        String fieldLayout = "";
        try {
            fieldLayout = new ObjectMapper().writeValueAsString(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
        } catch(Exception e) {
            e.printStackTrace();
        }
        fieldLayoutPub = NetworkTableInstance.getDefault()
                .getStringTopic("/gtsam_meme/input/tag_layout")
                .publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        fieldLayoutPub.set(fieldLayout);
        cameraNames.stream().map(CameraInterface::new).forEach(it -> cameras.put(it.name, it));

        loopTimeSub = NetworkTableInstance.getDefault()
                .getDoubleTopic("/gtsam_meme/output/loop_time_ms")
                .subscribe(0.0, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        readyToOptimizeSub = NetworkTableInstance.getDefault()
                .getBooleanTopic("/gtsam_meme/output/ready_to_optimize")
                .subscribe(false, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        hadIssueSub = NetworkTableInstance.getDefault()
                .getBooleanTopic("/gtsam_meme/output/had_issue")
                .subscribe(false, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
    }

    /**
     * Update the core camera intrinsic parameters. The localizer will apply these
     * as soon as reasonable, and makes no attempt to latency compensate this.
     * 
     * @param camName    The name of the camera
     * @param intrinsics Camera intrinsics in standard OpenCV format. See:
     *                   https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
     * @param distCoeffs Camera distortion coefficients, of length 4, 5 or 8
     */
    public void setCamIntrinsics(String camName, Optional<Matrix<N3, N3>> intrinsics,
            Optional<Matrix<N8, N1>> distCoeffs) {
        if (intrinsics.isEmpty() || distCoeffs.isEmpty()) {
            return;
        }

        var cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }

        final int distCoeffsSize = 8;
        double[] out = new double[distCoeffsSize + 4];
        out[0] = intrinsics.get().get(0, 0);
        out[1] = intrinsics.get().get(1, 1);
        out[2] = intrinsics.get().get(0, 2);
        out[3] = intrinsics.get().get(1, 2);
        for (int i = 0; i < distCoeffsSize; i++) {
            out[i + 4] = distCoeffs.get().get(i, 0);
        }
        cam.camIntrinsicsPublisher.set(out);
    }

    /**
     * Update the localizer with new info from this robot loop iteration.
     * 
     * @param odomTime The time that the odometry twist from last iteration
     *                 was collected at, in microseconds. WPIUtilJNI::now is
     *                 what I used
     * @param odom     The twist encoding chassis movement from last
     *                 timestamp to now. I use
     *                 SwerveDriveKinematics::toTwist2d
     */
    public void sendOdomUpdate(long odomTime, Twist3d odom) {

        // System.out.println("GtsamInterface.sendOdomUpdate time: " + odomTime);
        odomPub.set(odom, odomTime);

        localOdometryPose = localOdometryPose.exp(odom);
        odometryBuffer.addSample(odomTime / 1e6, localOdometryPose);
    }

    /**
     * Update the localizer with a new prior. Only updates if not ready to optimize
     * by default.
     * @param guess    An initial guess at robot
     *                 pose from solvePNP or prior knowledge.
     */
    public void sendPrior(long guessTime, Pose3d guess, boolean evenIfReadyToOptimize) {
        if((isConnected() && !isReadyToOptimize()) || evenIfReadyToOptimize) {
            Logger.recordOutput("Vision/Gtsam/Prior", guess);
            guessPub.set(guess, guessTime);
        }
    }

    /**
     * Update the localizer with new info from this robot loop iteration.
     * 
     * @param camName         The name of the camera
     * @param observation     The vision observation
     */
    public void sendVisionUpdate(String camName, EstimatedRobotPose observation) {

        var cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }

        long timestamp = Math.round(observation.timestampSeconds * 1e6);
        TagDetection[] tags = new TagDetection[observation.targetsUsed.size()];
        for (int i = 0; i < tags.length; i++) {
            PhotonTrackedTarget target = observation.targetsUsed.get(i);
            tags[i] = new TagDetection(target.getFiducialId(), target.getDetectedCorners());            
        }

        cam.tagPub.set(tags, timestamp);
    }

    public void sendRobotToCam(String camName, Transform3d robotTcam) {
        var cam = cameras.get(camName);
        if (cam == null) {
            throw new RuntimeException("Camera " + camName + " not in map!");
        }
        cam.robotTcamPub.set(robotTcam);
    }

    public Pose3d getRawPoseEstimate() {
        var poseEst = optimizedPoseSub.getAtomic();
        if (poseEst.timestamp != 0) {
            return poseEst.value;
        } else {
            // System.err.println("No pose estimate yet");
            return new Pose3d();
        }
    }

    public Pose3d getLatencyCompensatedPoseEstimate() {
        var poseEst = optimizedPoseSub.getAtomic();
        if (poseEst.timestamp != 0) {
            var poseAtSample = odometryBuffer.getSample(poseEst.timestamp / 1e6);
            var poseNow = localOdometryPose;

            if (poseAtSample.isEmpty()) {
                // huh
                System.err.println("pose outside buffer?");
                return new Pose3d();
            }

            var poseDelta = poseNow.minus(poseAtSample.get());
            return poseEst.value.transformBy(poseDelta);
        } else {
            // System.err.println("No pose estimate yet");
            return new Pose3d();
        }
    }

    public double[] getPoseStddevs() {
        return poseStddevSub.get();
    }

    // public Pose3d[] getOptimizedTraj() {
    //     return optimizedTrajSub.get();
    // }

    public double getLoopTimeMs() {
        return loopTimeSub.get();
    }

    public boolean isReadyToOptimize() {
        return readyToOptimizeSub.get();
    }

    public boolean isConnected() {
        var conns = NetworkTableInstance.getDefault().getConnections();
        for (var con : conns) {
            if (con.remote_id.indexOf("gtsam-meme") != -1) { 
                return true;
            }
        }
        return false;
    }

    public boolean hadIssue() {
        return hadIssueSub.get();
    }
}