package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.gyro.Gyro;
import frc.robot.subsystems.swerve.gyro.GyroRedux;
import frc.robot.subsystems.swerve.gyro.GyroSim;
import frc.robot.subsystems.swerve.setpointgen.ModuleLimits;
import frc.robot.subsystems.swerve.setpointgen.SwerveSetpoint;
import frc.robot.subsystems.swerve.setpointgen.SwerveSetpointGenerator;
import frc.robot.subsystems.vision.Vision;

public class Swerve extends SubsystemBase {

  static final Lock odometryLock = new ReentrantLock();
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(WHEEL_POSITIONS);
  SwerveDrivePoseEstimator multitagPoseEstimator;
  SwerveDriveOdometry wheelOdometry;
  Field2d field = new Field2d();
  Vision vision;

  Gyro gyro;
  PIDController headingController = new PIDController(5.2, 0, 0.5);
  SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(kinematics, WHEEL_POSITIONS);

  SwerveModule[] modules = {
    new SwerveModule(0, "FL"),
    new SwerveModule(1, "FR"),
    new SwerveModule(2, "BL"),
    new SwerveModule(3, "BR")
  };

  PIDController xPid = new PIDController(XY_PATH_FOLLOWING_PID[0], XY_PATH_FOLLOWING_PID[1], XY_PATH_FOLLOWING_PID[2]);
  PIDController yPid = new PIDController(XY_PATH_FOLLOWING_PID[0], XY_PATH_FOLLOWING_PID[1], XY_PATH_FOLLOWING_PID[2]);
  PIDController rotPid = new PIDController(ROT_PATH_FOLLOWING_PID[0], ROT_PATH_FOLLOWING_PID[1], ROT_PATH_FOLLOWING_PID[2]);

  SwerveSetpoint lastSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
  ModuleLimits limits = new ModuleLimits(MAX_ROBOT_TRANS_SPEED, MAX_ROBOT_ACCEL, MAX_MOD_STEER_VEL);
  double lastTime = Timer.getTimestamp();

  public Swerve() {
    this.vision = new Vision(this);
    if (Robot.isSimulation()) {
      gyro = new GyroSim();
    } else {
      gyro = new GyroRedux();
    }
    gyro.reset();
    SmartDashboard.putData("Field", field);
    
    for (SwerveModule module : modules) {
      module.configure();
    }
    
    SparkOdometryThread.getInstance().start();
    
    multitagPoseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), new Pose2d());
    wheelOdometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), getModulePositions());
    headingController.enableContinuousInput(0, 2 * Math.PI);
    headingController.setTolerance(HEADING_CONTROLLER_TOLERANCE);

    rotPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Rotation2d getGyroRotation() {
    return gyro.getRotation2d();
  }

  public double getGyroVelocity() {
    return gyro.getVelocityYaw();
  }

  @Override
  public void simulationPeriodic() {
    double rotationSpeed = getRobotRelativeSpeeds().omegaRadiansPerSecond;
    if(gyro instanceof GyroSim) {
      ((GyroSim) gyro).setVelocityYaw(rotationSpeed);
      ((GyroSim) gyro).setYaw(gyro.getRotation2d().getRadians() + rotationSpeed * 0.02);
    }
  }


  public void drive(ChassisSpeeds speeds, Double alignToAngle) {
    double commandedRot = headingController.calculate(getPose().getRotation().getRadians());

    if (alignToAngle != null) {
      Logger.recordOutput("Swerve/PoseRotPidAtSetpoint", headingController.atSetpoint());
      headingController.setSetpoint(alignToAngle);
      if (!headingController.atSetpoint()) {
        speeds.omegaRadiansPerSecond = commandedRot;
      } else {
        speeds.omegaRadiansPerSecond = 0;
      }
    }

    Logger.recordOutput("Swerve/Speeds", speeds);

    // SwerveDriveKinematics.desaturateWheelSpeeds(getModuleStates(), speeds, MAX_MOD_SPEED, MAX_ROBOT_TRANS_SPEED, MAX_ROBOT_ROT_SPEED);
    Logger.recordOutput("Swerve/DesaturatedSpeeds", speeds);

    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);
    Logger.recordOutput("Swerve/DiscretizedSpeeds", discretized);

    // lastSetpoint = setpointGenerator.generateSetpoint(limits, lastSetpoint, speeds, Timer.getFPGATimestamp() - lastTime);
    // speeds = lastSetpoint.chassisSpeeds();
    // lastTime = Timer.getFPGATimestamp();
    // Logger.recordOutput("Swerve/SetpointGeneratedSpeeds", speeds);
    // SwerveModuleState[] moduleStates = lastSetpoint.moduleStates();

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discretized);
    Logger.recordOutput("Swerve/DesiredStates", moduleStates);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, null);
  }

  /* relative to your alliance's DS wall */
  public void driveAllianceRelative(ChassisSpeeds speeds, Double alignToAngle) {
    var rot = getPose().getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      rot = rot.rotateBy(Rotation2d.fromDegrees(180));
    }
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot), alignToAngle);
  }

  /* relative to blue ds wall */
  public void driveFieldRelative(ChassisSpeeds speeds, Double alignToAngle) {
    var rot = getPose().getRotation();
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rot), alignToAngle);
  }

  public void driveAllianceRelative(ChassisSpeeds speeds) {
    driveAllianceRelative(speeds, null);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    driveFieldRelative(speeds, null);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getCurrentModulePosition();
    }
    Logger.recordOutput("Swerve/Positions", positions);
    return positions;
  }

  @AutoLogOutput
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    Logger.recordOutput("Swerve/States", states);
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return multitagPoseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    multitagPoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    wheelOdometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }

  public double getCurrentDraw() {
    double totalCurrent = 0;
    for (SwerveModule module : modules) {
      totalCurrent += module.drive.getCurrent();
      totalCurrent += module.steer.getCurrent();
    }
    return totalCurrent;
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public void periodic() {
    // gyro.log();
    for (SwerveModule module : modules) {
      module.periodic();
    }
    if (Robot.isSimulation()) {
      wheelOdometry.update(getGyroRotation(), getModulePositions());
      Logger.recordOutput("Swerve/PoseWheelsOnly", getPoseWheelsOnly());
    }
    vision.feedPoseEstimator(multitagPoseEstimator);
    multitagPoseEstimator.update(getGyroRotation(), getModulePositions());
    field.setRobotPose(getPose());
  }

  public Pose2d getPoseWheelsOnly() {
    return wheelOdometry.getPoseMeters();
  }

  public void followTrajectory(SwerveSample sample) {
    var pose = getPose();

    ChassisSpeeds speeds = new ChassisSpeeds(
      sample.vx + xPid.calculate(pose.getX(), sample.x),
      sample.vy + yPid.calculate(pose.getY(), sample.y),
      sample.omega + rotPid.calculate(pose.getRotation().getRadians(), sample.heading)
    );

    Logger.recordOutput("Swerve/Traj/Sample", sample);
    driveFieldRelative(speeds);
  }


  public void setDriveVoltage(Voltage volts) {
    for (SwerveModule mod : modules) {
      mod.setDriveVolts(volts.magnitude());
    }
  }

  public void setSteerVoltage(Voltage volts) {
    for (SwerveModule mod : modules) {
      mod.setSteerVolts(volts.magnitude());
    }
  }

  public SysIdRoutine getDriveSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(5.5), null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism((Voltage v) -> {
        modules[0].steer.setAngle(new Rotation2d());
        modules[1].steer.setAngle(new Rotation2d());
        modules[2].steer.setAngle(new Rotation2d());
        modules[3].steer.setAngle(new Rotation2d());
        setDriveVoltage(v);
      }, null, this)
    );
  }

  public SysIdRoutine getAngularSysIdRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(3.5), null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())), 
      new SysIdRoutine.Mechanism((Voltage v) -> {
        var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 100));
        for(int i = 0; i < modules.length; i++) {
          modules[i].steer.setAngle(states[i].angle);
        }
        setDriveVoltage(v);
      }, null, this)
    );
  }
}
