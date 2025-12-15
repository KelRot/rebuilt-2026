// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util.Motors;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public MotorIOData motorData = new MotorIOData(false, 0, null, false);
    }

    record MotorIOData(boolean isConnected, int canID, MotorType motorType, boolean hasError) {}

    enum MotorType { 
        TALON,
        SPARKMAX,
        SPARKFLEX
    }

    public default void updateInputs(MotorIOInputs inputs) {}

    public default void setOpenLoop(double output) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void setCoastMode(boolean coastMode) {}

    public default void applyConfig(MotorConfig config) {}


}
