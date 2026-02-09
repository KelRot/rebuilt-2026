package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    @AutoLog
    public static class KickerIOInputs {
        public boolean rollerConnected = false;
        public double rollerMotorCurrentAmps = 0.0;
        public double rollerMotorVoltageVolts = 0.0;
        public double rollerMotorSpeedRpm = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {
    }

    public default void setRollerVoltage(double volts) {
    }

    public default void stopAllMotors() {
    }

}
