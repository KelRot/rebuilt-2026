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
        public static final int kLedLength = 20;
        public static final Distance kLedSpacing = Meters.of(0.02);
    }

    public static class TurretConstants {
        public static final double kP = 0.0;
        public static final double kD = 0.0;

        public static final int turretID = 9;
        public static final int absEncoder1ID = 3;
        public static final int absEncoder2ID = 0;

        public static final double positionConversionFactor = 360/18;
        public static final double velocityConversionFactor = 1;

        public static final DCMotor turretGearbox = DCMotor.getNEO(1);
        public static final double turretMotorReduction = 18;

        public static final double minAngle = 0;
        public static final double maxAngle = 360;

        public static final double cruiseVelocity = 0.0;
        public static final double maxAcceleration = 0.0;

        public static final int turretGearboxTeeth = 108;
        public static final int absEncoder1Teeth = 24;
        public static final int absEncoder2Teeth = 25;
        public static final double abs1Offset = 0.0;
        public static final double abs2Offset = 0.0;

        public static final Translation2d turretOffset = new Translation2d(0.135, 0.0);
    }

    public static class IntakeConstants {
        public static final int rollerMotorID = 0;
        public static final int openerMotorID = 3;
        public static final int secondOpenerMotorID = 7;
        public static final double zeroWaitSeconds = 2;
        public static final double zeroVoltage = -2.5;
        public static double openerGearRatio = 112 / 14 * 23 / 13;
        public static double INTAKING_RPM = 2000.0;
        public static double OUTTAKING_RPM = -1800.0;
        public static double intakeOpenPosition = 85;
        public static double openVoltage;
        public static Time openWaitSeconds;
        public static double intakeClosedPosition = -34;
        public static final double ZERO_CONFIRM_TIME = 0.5;
        public static final int ZERO_VELOCITY_EPS = 3;
        public static final double RotPerVolt = 5906 / 12;
        public static final double intakeLength = 0.0;
    }

    public static class IndexConstants {
        public static final int spinnerMotorID = 4;

        public static final double PASSIVE_MODE_VOLTAGE = -1.0;
        public static final double INDEXING_VOLTAGE = -12 * 0.6;
        public static final double OUTTAKING_VOLTAGE = 12 * 0.3;

    }

    public static class KickerConstants {
        public static final int kickerMotorID = 8;
        public static final double defaultKickerVoltage = -5.5;
    }

    public static class FlywheelConstants {
        public static final int kMasterMotorId = 1;
        public static final int kFollowerMotorId = 2;
        public static final double RotPerVolt = 580;
    }

    public static final class HoodConstants {

        public static final int hoodID = 5;

        public static final double kP = 0.0;
        public static final double kD = 0.0;

        public static final double positionConversionFactorDeg = 1 / (27 * 48 / 15) / 360;

        public static final double cruiseVelocityDegPerSec = 0.0;

        public static final double maxAccelerationDegPerSec2 = 0.0;

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