package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int modIndex;
    private final String modulePath;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public SwerveModule(SwerveModuleIO io, int modIndex, String moduleName) {
        this.modulePath = "Swerve/" + moduleName;
        this.io = io;
        this.modIndex = modIndex;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(modulePath, inputs);
        
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
          double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
          Rotation2d angle = inputs.odometryTurnPositions[i];
          odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    public void setState(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        
        io.setDriveVelocity(state.speedMetersPerSecond);
        io.setAngle(state.angle);
    }

    public void stop() {
        io.setDriveVolts(0);
        io.setSteerVolts(0);
    }

    public Rotation2d getAngle() {
        return inputs.steerAngle;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * WHEEL_RADIUS;
    }   

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public void configure() {
        io.configureDrive();
        io.configureSteer();
    }

}
