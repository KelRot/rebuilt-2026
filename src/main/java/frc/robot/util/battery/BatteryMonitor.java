// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.battery;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class BatteryMonitor extends Command {

    private static final int SAMPLE_COUNT = 50;
    private static final double MIN_VOLTAGE = 12.35;
    private static final String NT_TOPIC = "Battery/";

    private final LinearFilter filter;
    private final LoggedNetworkBoolean lowBattery;
    private final Alert alert;
    private double avgVoltage;

    public BatteryMonitor() {
        filter = LinearFilter.movingAverage(SAMPLE_COUNT);
        lowBattery = new LoggedNetworkBoolean(NT_TOPIC + "Low");
        alert = new Alert("Battery is too low for the match", AlertType.kWarning);

        avgVoltage = BatteryUtils.getCurrentVoltage();
    }

    @Override
    public void initialize() {
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            filter.calculate(avgVoltage);
        }
    }

    @Override
    public void execute() {
        avgVoltage = filter.calculate(BatteryUtils.getCurrentVoltage());

        boolean isLow = avgVoltage <= MIN_VOLTAGE;

        if (isLow && !DriverStation.isEnabled()) {
            lowBattery.set(true);
            alert.setText(String.format("Battery is too low for the match : %2fV", avgVoltage));
            alert.set(true);
        } else {
            lowBattery.set(false);
            alert.set(false);
        }
    }

    public boolean isFinished() {
        return DriverStation.isEnabled();
    }
}
