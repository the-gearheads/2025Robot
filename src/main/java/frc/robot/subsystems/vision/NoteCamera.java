// package frc.robot.subsystems.vision;


// import org.littletonrobotics.junction.Logger;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;

// public class NoteCamera {

//   Transform3d offset;
//   PhotonCamera camera;
//   String path;

//   private final double MIN_AREA = 1;

//   public NoteCamera(String name, Transform3d offset) {
//     // This is a constructor
//     camera = new PhotonCamera(name);
//     this.offset = offset;
//     path = "Vision/" + name.replace("_", "");
//   }

//   public PhotonTrackedTarget getTarget() {
//     PhotonPipelineResult res = camera.getLatestResult();
//     int numTargets = res.getTargets().size();
//     Logger.recordOutput(path + "/Valid", numTargets > 0);
//     Logger.recordOutput(path + "/NumTargets", numTargets);

//     PhotonTrackedTarget bestNoteTarget = numTargets > 0 ? res.getTargets().get(0) : null;
//     if(bestNoteTarget != null) {
//       Logger.recordOutput(path + "/Note", bestNoteTarget.getBestCameraToTarget());
//       Logger.recordOutput(path + "/Yaw", bestNoteTarget.getYaw());
//       Logger.recordOutput(path + "/Pitch", bestNoteTarget.getPitch());
//       Logger.recordOutput(path + "/Area", bestNoteTarget.getArea());
//       Logger.recordOutput(path + "/Skew", bestNoteTarget.getSkew());

//       if(bestNoteTarget.getArea() < MIN_AREA) {
//         return null;
//       }
//     } else {
//       Logger.recordOutput(path + "/Yaw", 0d);
//       Logger.recordOutput(path + "/Pitch", 0d);
//       Logger.recordOutput(path + "/Area", 0d);
//       Logger.recordOutput(path + "/Skew", 0d);
//     }

//     return bestNoteTarget;
//   }

//   public Transform3d getCameraOffset() {
//     return offset;
//   }

//   public String getPath() {
//     return path;
//   }

//   public void logCamTransform(Pose2d robotPose) {
//     Pose3d camPose = new Pose3d(robotPose);
//     camPose = camPose.transformBy(getCameraOffset());
//     Logger.recordOutput(path + "/CamTransform", camPose);
//   }

// }
