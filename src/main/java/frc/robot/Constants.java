// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final int PDH_ID = 1;
    public static final boolean tuningMode = true;

    public static class LedConstants {
        public static final int kLedPort = 0;
        public static final int kLedLength = 24;
        public static final Distance kLedSpacing = Meters.of(0.05);
    }

    public static class TurretConstants {
        public static final double kP = 0.0;
        public static final double kD = 0.0;
    
        public static final int turretID = 0;
        public static final int absEncoder1ID = 0;
        public static final int absEncoder2ID = 0;

        public static final double positionConversionFactor = 0; // Example conversion factor
        public static final double velocityConversionFactor = 0; 

        public static final DCMotor turretGearbox = DCMotor.getNEO(0);
        public static final double turretMotorReduction = 0;

        public static final double absolutePositionOffsetRads1 = 0.0;
        public static final double absolutePositionOffsetRads2 = 0.0;

        public static final double minAngle = -270;
        public static final double maxAngle = 270;

        public static final double cruiseVelocity = 0.0;
        public static final double maxAcceleration = 0.0;

        public static final double zoneLine = 0.0;

        public static final int turretGearboxTeeth = 0;
        public static final int absEncoder1Teeth = 0;
        public static final int absEncoder2Teeth = 0;
        public static final double abs1Offset = 0.0;
        public static final double abs2Offset = 0.0;
    }

    

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}

