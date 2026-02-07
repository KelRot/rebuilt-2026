package frc.robot.subsystems.flywheel;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    private final FlyWheelIO io;
    private final FlyWheelIO.FlywheelIOInputs inputs =
            new FlyWheelIO.FlywheelIOInputs();

    public Flywheel(FlyWheelIO io) {
        this.io = io;
    }

    // tune
    private static final TreeMap<Double, Double> distanceToRPM = new TreeMap<>();

    static {
        distanceToRPM.put(1.5, 2800.0);
        distanceToRPM.put(2.0, 3100.0);
        distanceToRPM.put(2.5, 3500.0);
        distanceToRPM.put(3.0, 3900.0);
        distanceToRPM.put(3.5, 4300.0);
    }

    public static double getRPMForDistance(double distanceMeters) {
        var low = distanceToRPM.floorEntry(distanceMeters);
        var high = distanceToRPM.ceilingEntry(distanceMeters);

        if (low == null) {
            return distanceToRPM.firstEntry().getValue();
        }

        if (high == null) {
            return low.getValue();
        }

        if (low.getKey().equals(high.getKey())) {
            return low.getValue();
        }

        // Linear interpolation
        double t = (distanceMeters - low.getKey())
                 / (high.getKey() - low.getKey());

        return low.getValue() + t * (high.getValue() - low.getValue());
    }

    public void setRPMFromDistance(double distanceMeters) {
        double rpm = getRPMForDistance(distanceMeters);
        io.setRpm(rpm);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double rpm =
            inputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);

        SmartDashboard.putNumber("Flywheel RPM", rpm);
        SmartDashboard.putNumber("Flywheel Applied Voltage", inputs.appliedVoltage);
        SmartDashboard.putNumber("Flywheel Output Current (Amps)", inputs.outputCurrentAmps);
        SmartDashboard.putNumber("Flywheel Lead Target RPM", inputs.leadTargetRpm);
    }
}
