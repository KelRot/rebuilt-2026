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

package frc.robot.util.motors;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public MotorIOData motorData = new MotorIOData(false, 0, null, false);
    }

    record MotorIOData(boolean isConnected, int canID, ControllerType controllerType, boolean hasError) {}

    enum ControllerType { 
        TALON,
        SPARKMAX,
        SPARKFLEX
    }
    
    record Motor(ControllerType controllerType, int canID) {

}

    public default void updateInputs(MotorIOInputs inputs) {}

}
