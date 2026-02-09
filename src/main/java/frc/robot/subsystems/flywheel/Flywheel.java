package frc.robot.subsystems.flywheel;

import java.util.TreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    /* ---------------- State ---------------- */

    public enum FlywheelState {
        IDLE,
        SHOOTING
    }

    private FlywheelState state = FlywheelState.IDLE;
    private double targetDistanceMeters = 0.0;

    /* ---------------- IO ---------------- */

    private final FlywheelIO io;
    private final FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    /* ---------------- State setters ---------------- */

    public void setState(FlywheelState state) {
        this.state = state;
    }

    public void setTargetDistanceMeters(double distanceMeters) {
        this.targetDistanceMeters = distanceMeters;
    }

    /* ---------------- Distance → RPM mapping ---------------- */

    private static final TreeMap<Double, Double> kDistanceToRpmMap = new TreeMap<>();

    static {
        kDistanceToRpmMap.put(1.5, 2800.0);
        kDistanceToRpmMap.put(2.0, 3100.0);
        kDistanceToRpmMap.put(2.5, 3500.0);
        kDistanceToRpmMap.put(3.0, 3900.0);
        kDistanceToRpmMap.put(3.5, 4300.0);
    }

    public static double calculateRpmForDistance(double distanceMeters) {
        var low = kDistanceToRpmMap.floorEntry(distanceMeters);
        var high = kDistanceToRpmMap.ceilingEntry(distanceMeters);

        if (low == null) {
            return kDistanceToRpmMap.firstEntry().getValue();
        }

        if (high == null) {
            return low.getValue();
        }

        if (low.getKey().equals(high.getKey())) {
            return low.getValue();
        }

        double t = (distanceMeters - low.getKey())
                / (high.getKey() - low.getKey());

        return low.getValue() + t * (high.getValue() - low.getValue());
    }

    /* ---------------- Periodic ---------------- */

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (state) {
            case SHOOTING -> {
                double targetRpm = calculateRpmForDistance(targetDistanceMeters);
                io.setRpm(targetRpm);
            }

            case IDLE -> {
                // Idle RPM to prevent stalling
                io.setRpm(500.0);
            }
        }
    }
}
