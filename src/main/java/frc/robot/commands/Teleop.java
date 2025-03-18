package frc.robot.commands;

import static frc.robot.constants.MiscConstants.AUTO_ALIGN_ANGLE_THRESHOLD;
import static frc.robot.constants.MiscConstants.AUTO_ALIGN_DIST_THRESHOLD;
import static frc.robot.constants.SwerveConstants.MAX_ROBOT_TRANS_SPEED;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlignToPose;

public class Teleop extends Command {
    Swerve swerve;
    AlignToPose autoAlign = new AlignToPose();
    Vision vision;
    
    public Teleop(Swerve swerve, Vision vision) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.vision = vision;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = Controllers.driverController.getTranslateXAxis();
        double y = Controllers.driverController.getTranslateYAxis();
        double rot = Controllers.driverController.getRotateAxis();

        x = Math.signum(x) * Math.pow(x, 2);
        y = Math.signum(y) * Math.pow(y, 2);
        rot = Math.signum(rot) * Math.pow(rot, 2);

        x *= MAX_ROBOT_TRANS_SPEED;
        y *= MAX_ROBOT_TRANS_SPEED;
        rot *= MAX_ROBOT_TRANS_SPEED;

        ChassisSpeeds finalSpeeds;
        // decide whether to do autoalign
        Rotation2d controllerAngle = new Translation2d(x, y).getAngle();
        Pose2d currentCoralTarget = autoAlign.getCoralObjective(swerve.getPose(), controllerAngle);
        if(currentCoralTarget.getTranslation().getDistance(swerve.getPose().getTranslation()) < AUTO_ALIGN_DIST_THRESHOLD && Math.abs(currentCoralTarget.getRotation().minus(swerve.getPose().getRotation()).getRadians()) < AUTO_ALIGN_ANGLE_THRESHOLD) {
            Logger.recordOutput("AlignToPose/TeleopAligning", true);
            vision.setPoseStrategy(1, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            // vision.setPoseStrategy(2, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            Pair<ChassisSpeeds, Double> autoAlignSpeeds = autoAlign.getAutoAlignSpeeds(x, y, swerve.getPose());
            ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x * (1 - autoAlignSpeeds.getSecond()), y * (1 - autoAlignSpeeds.getSecond()), rot * (1 - autoAlignSpeeds.getSecond())), swerve.getPose().getRotation());
            finalSpeeds = driverSpeeds.plus(autoAlignSpeeds.getFirst());
            // finalSpeeds = driverSpeeds;
        } else {
            Logger.recordOutput("AlignToPose/TeleopAligning", false);
            vision.defaultPoseStrategies();
            ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(x, y, rot), swerve.getPose().getRotation());
            finalSpeeds = driverSpeeds;
        }

        swerve.drive(finalSpeeds);
    }
}
