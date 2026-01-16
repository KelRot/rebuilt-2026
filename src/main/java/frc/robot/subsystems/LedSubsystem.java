// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.Led;

public class LedSubsystem extends SubsystemBase {
        @SuppressWarnings("unchecked")
        public LedSubsystem(Led led) {
                led.setStaticColor(Color.kAliceBlue);;
        }
}
