// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.battery;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class BatteryUtils {

    public static final double DEFAULT_VOLTAGE = 12.35;

    private static final PowerDistribution powerDistribution = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

    private static final Command monitor = new BatteryMonitor().ignoringDisable(true);

    public static double getTotalCurrent() {
        return powerDistribution.getTotalCurrent();
    }

    public static double getCurrentVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public static void scheduleMonitor() {
        if (!monitor.isScheduled()) {
            CommandScheduler.getInstance().schedule(monitor);
        }
    }
}
