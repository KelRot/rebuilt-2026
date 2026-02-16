// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.5; // 16.9 ft/s = 5.15 m/s
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.868928);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.556138);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.5306);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(1.26);

    // Device CAN IDs
    public static final int pigeonCanId = 31;

    public static final int frontLeftDriveCanId = 11;
    public static final int backLeftDriveCanId = 13;
    public static final int frontRightDriveCanId = 14;
    public static final int backRightDriveCanId = 59;

    public static final int frontLeftTurnCanId = 23;
    public static final int backLeftTurnCanId = 3;
    public static final int frontRightTurnCanId = 22;
    public static final int backRightTurnCanId = 24;

    public static final int frontLeftTurnAbsId = 3;
    public static final int backLeftTurnAbsId = 1;
    public static final int frontRightTurnAbsId = 2;
    public static final int backRightTurnAbsId = 0;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60; // 60 looks like a good but can be lower. Look at later.
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction = 6.0;
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
    // ->
    // Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0005; // Tune later
    public static final double driveKd = 0.00000000028;
    public static final double driveKs = 0.07735;
    public static final double driveKv = 0.10215;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 25.0;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = false;
    public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 25;
    public static final double robotMOI = 3;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);
}


// ks = 0.07903
// kv = 0.10014
//