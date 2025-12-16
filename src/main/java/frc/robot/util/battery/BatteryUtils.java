package frc.robot.util.battery;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
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
			monitor.schedule();
		}
	}

}