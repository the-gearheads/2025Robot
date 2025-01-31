// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file in
// the directory of this file.

package frc.robot.subsystems.swerve.setpointgen;

public record ModuleLimits(
    double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}
