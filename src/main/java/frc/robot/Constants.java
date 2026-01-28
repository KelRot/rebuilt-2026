// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

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

     public static class KickerConstants {
         public static final int rollerMotorID = 4;
        public static final double zeroWaitSeconds = 2;
        public static final double zeroVoltage = 0;
        public static double openerGearRatio = 1.0;
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
