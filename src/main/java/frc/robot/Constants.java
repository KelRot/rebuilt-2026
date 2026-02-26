// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
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
    
        public static final int turretID = 1;
        public static final int absEncoder1ID = 2;
        public static final int absEncoder2ID = 3;

        public static final double positionConversionFactor = 1; // Example conversion factor
        public static final double velocityConversionFactor = 1; 

        public static final DCMotor turretGearbox = DCMotor.getNEO(1);
        public static final double turretMotorReduction = 25;

        public static final double minAngle = -270;
        public static final double maxAngle = 270;

        public static final double cruiseVelocity = 0.0;
        public static final double maxAcceleration = 0.0;

        public static final int turretGearboxTeeth = 220;
        public static final int absEncoder1Teeth = 25;
        public static final int absEncoder2Teeth = 24;
        public static final double abs1Offset = 0.0;
        public static final double abs2Offset = 0.0;

        public static final Translation2d turretOffset = new Translation2d(0.0, 0.0);
    }

    
    public static class IntakeConstants {
        public static final int rollerMotorID = 0;
        public static final int openerMotorID = 1;
        public static final double zeroWaitSeconds = 2;
        public static final double zeroVoltage = 0;
        public static double openerGearRatio = 3.2;
        public static double INTAKING_VOLTAGE;
        public static double OUTTAKING_VOLTAGE;
        public static double intakeOpenPosition = -133;
        public static double openVoltage;
        public static Time openWaitSeconds;
        public static double intakeClosedPosition = 0;
        public static final int secondOpenerMotorID = 2;
        public static final double ZERO_CONFIRM_TIME = 0;
        public static final int ZERO_VELOCITY_EPS = 0;
    }

    public static class IndexConstants {
        public static final int spinnerMotorID = 3;

        public static final double PASSIVE_MODE_VOLTAGE = 3.0;
        public static final double INDEXING_VOLTAGE = 10.0;
        public static final double OUTTAKING_VOLTAGE = -10.0;
    }

    public static class KickerConstants {
        public static final int kickerMotorID = 4;
        public static final double defaultKickerVoltage = 0;
    }

    public static class FlywheelConstants {
        public static final int kMasterMotorId = 7;
        public static final int kFollowerMotorId = 8;
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
    }

    public static final class HoodConstants {

        public static final int hoodID = 0;

        public static final double kP = 0.0;
        public static final double kD = 0.0;

        public static final double positionConversionFactorDeg = 1.0;

        public static final double cruiseVelocityDegPerSec = 0.0;

        public static final double maxAccelerationDegPerSec2 = 0.0;

        public static final double minAngleDeg = 0.0;
        public static final double maxAngleDeg = 0.0;

    }

    public static final class FieldConstants {

        public static final AprilTagFields DEFAULT_APRILTAG_TYPE =
        AprilTagFields.k2026RebuiltWelded;

        public static final int aprilTagCount = 32;
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

