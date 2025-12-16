package frc.robot.util.battery;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class BatteryMonitor extends Command {

    private static final int SAMPLE_COUNT = 50;
    private static final double MIN_VOLTAGE = 12.35;
    private static final String NT_TOPIC = "Battery/";

    private final LinearFilter filter;
    private final LoggedNetworkBoolean lowBattery;
    private final LoggedNetworkNumber voltage;
    private double avgVoltage;

    public BatteryMonitor() {
        filter = LinearFilter.movingAverage(SAMPLE_COUNT);
        lowBattery = new LoggedNetworkBoolean(NT_TOPIC + "Low");
        voltage = new LoggedNetworkNumber(NT_TOPIC + "Voltage");

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
            voltage.set(avgVoltage);
        } else {
            lowBattery.set(false);
        }
    }
}
