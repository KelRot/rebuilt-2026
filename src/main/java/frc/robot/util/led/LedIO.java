// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
    @AutoLog
    public static class LedIOInputs {
        public boolean connected = false;
        public String activePatternName = "";
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(LedIOInputs inputs) {}

    /** Sets the data to the LED strip. */
    default void setData(AddressableLEDBuffer buffer) {}
}
