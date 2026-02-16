package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    @AutoLog
    public static class KickerIOInputs {
        public boolean kickerConnected = false;
        public double kickerMotorCurrentAmps = 0.0;
        public double kickerMotorVoltageVolts = 0.0;
        public double kickerMotorSpeedRpm = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {
    }

    public default void setKickerVoltage(double volts) {
    }

    public default void stopAllMotors() {
    }

}
