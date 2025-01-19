package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTemperature = 0.0;

        public Rotation2d steerAngle;  // no offsetting done yet
        public double steerVelocityRadPerSec = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};    
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void configureDrive() {}

    public default void configureSteer() {}

    public default void setDriveVolts(double volts) {}

    public default void setSteerVolts(double volts) {}

    public default void resetDriveEncoder() {}

    public default void resetSteerEncoder() {}

    public default void setDriveVelocity(double velocityRadPerSec) {}

    public default void setAngle(Rotation2d rotation) {}
  
}