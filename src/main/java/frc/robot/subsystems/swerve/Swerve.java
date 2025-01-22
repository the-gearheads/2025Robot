package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.WHEEL_POSITIONS;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Gyro.GyroIO;
import frc.robot.subsystems.swerve.Gyro.GyroIOInputsAutoLogged;

public class Swerve extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  Field2d field = new Field2d();
  SwerveDrivePoseEstimator multitagPoseEstimator;

  public Swerve(GyroIO gyro, SwerveModuleIO flModule, SwerveModuleIO frModule, SwerveModuleIO blModule, SwerveModuleIO brModule) {
    this.gyro = gyro;
    modules[0] = new SwerveModule(flModule, 0, "Front Left");
    modules[1] = new SwerveModule(frModule, 0, "Front Right");
    modules[2] = new SwerveModule(blModule, 0, "Back Left");
    modules[3] = new SwerveModule(brModule, 0, "Back Right");

    // need to initalize pose est with wheel positions and gyro
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (SwerveModule module : modules) {
      module.periodic();
    }

    multitagPoseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroInputs.yaw, getModulePositions(), new Pose2d());
    SparkOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (SwerveModule module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      multitagPoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

  }

  public Pose2d getPose() {
    return multitagPoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    multitagPoseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  public void drive(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, );

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].setState(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }
  
  public void driveFieldRelative(ChassisSpeeds speeds) {
    var rot = getPose().getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      rot = rot.rotateBy(Rotation2d.fromDegrees(180));
    }
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot));
  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

}
