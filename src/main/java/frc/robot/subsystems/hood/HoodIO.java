package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        public boolean connected = false;
        public double positionDeg = 0.0;

        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {
    }

    default void setPosition(double positionDeg) {
    }

    default void stop() {
    }
    default void setAppliedVoltage(double volts) {
        }
    public default boolean isAtSetpoint() {
        return false;
    }
}
