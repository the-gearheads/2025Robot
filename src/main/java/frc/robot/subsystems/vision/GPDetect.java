package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;

public class GPDetect extends SubsystemBase {
  final NoteCamera NOTE_CAMERA = new NoteCamera(VisionConstants.NOTE_CAM_NAME, VisionConstants.NOTE_CAM_TRANSFORM);
  private Swerve swerve;

  public GPDetect(Swerve swerve) {
    this.swerve = swerve;
  }

  public Translation2d getNearestNote() {
    PhotonTrackedTarget bestNote = NOTE_CAMERA.getTarget();
    if (bestNote == null) {
      return null;
    }
    double targYaw = Units.degreesToRadians(bestNote.getYaw());
    double targPitch = Units.degreesToRadians(bestNote.getPitch());

    double noteDistY = (1/Math.tan(targPitch-NOTE_CAMERA.getCameraOffset().getRotation().getY()))* NOTE_CAMERA.getCameraOffset().getZ();
    double noteDistX = Math.tan(targYaw-NOTE_CAMERA.getCameraOffset().getRotation().getZ())*noteDistY;
    Translation2d noteTrans = new Translation2d(noteDistY, noteDistX);

    Logger.recordOutput(NOTE_CAMERA.getPath() + "/NoteTransform", swerve.getPose().transformBy(new Transform2d(noteTrans, new Rotation2d())).transformBy(new Transform2d(NOTE_CAMERA.getCameraOffset().getTranslation().toTranslation2d(), new Rotation2d())));
    return noteTrans;
  }

  @Override
  public void periodic() {
    NOTE_CAMERA.logCamTransform(swerve.getPose());
  }

}
