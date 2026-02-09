package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public boolean rollerConnected = false;
        public boolean openerConnected = false;
        public boolean secondOpenerConnected = false;
        public double rollerMotorCurrentAmps = 0.0;
        public double rollerMotorVoltageVolts = 0.0;
        public double rollerMotorSpeedRpm = 0.0;
        public double openerMotorVoltageVolts = 0.0;
        public double openerMotorCurrentAmps = 0.0;
        public double IntakePosition = 0.0;
        public boolean isIntakeOpen = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setRollerVoltage(double volts) {
    }

    public default void setOpenerSetPoint(double setPoint) {
    }

    public default void setOpenerVoltage(double volts) {
    }

    public default void zeroEncoder() {
    }

    public default void stopAllMotors() {
    }

    public default boolean isOpenerAtSetpoint() {
        return false;
    }
}