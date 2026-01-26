package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {

    @AutoLog
    public static class IndexIOInputs {
        public boolean spinnerConnected = false;
        public double spinnerMotorCurrentAmps = 0.0;
        public double spinnerMotorVoltageVolts = 0.0;
        public double spinnerMotorSpeedRpm = 0.0;
    }

    public default void updateInputs(IndexIOInputs inputs) {
    }

    public default void setSpinnerVoltage(double volts) {
    }

    public default void zeroEncoder() {
    }

    public default void stopAllMotors() {
    }

}