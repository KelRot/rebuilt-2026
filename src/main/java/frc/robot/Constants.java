// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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
    public static final String Intake = null;

    public static class LedConstants {
        public static final int kLedPort = 0;
        public static final int kLedLength = 24;
        public static final Distance kLedSpacing = Meters.of(0.05);
    }
    public static class IntakeConstants {
        public static final int rollerMotorID = 0;
        public static final int openerMotorID = 1;
        public static final double zeroWaitSeconds = 2;
        public static final double zeroVoltage = 0;
        public static double openerGearRatio = 3.2;
        public static double INTAKING_VOLTAGE;
        public static double OUTTAKING_VOLTAGE;
        public static double intakeOpenPosition;
        public static double openVoltage;
        public static Time openWaitSeconds;
        public static final int secondOpenerMotorID = 2;
    }

    public static class IndexConstants {
        public static final int spinnerMotorID = 3;

        public static final double PASSIVE_MODE_VOLTAGE = 3.0;
        public static final double INDEXING_VOLTAGE = 10.0;
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
