package frc.robot.subsystems.intake;

import java.security.GeneralSecurityException;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public boolean rollerConnected = false;
        public boolean openerConnected = false;
        public boolean secondOpenerConnected = false;
        public double rollerMotorCurrentAmps = 0.0;
        public double rollerMotorVoltageVolts = 0.0;
        public double rollerMotorSpeedRpm = 0.0;
        public double openerMotorVelocityRPS = 0.0;
        public double openerMotorVoltageVolts = 0.0;
        public double openerMotorCurrentAmps = 0.0;
        public double IntakePosition = 37.0;
        public boolean isIntakeOpen = false;
        public boolean isZeroed = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setRollerVoltage(double volts) {
    }

    public default void setOpenerSetPoint(double setPoint, ControlType controlType) {
    }

    public default void setRollerRPM(double rpm) {
    }

    public default SparkBase getLeadOpenerMotor() {
        return null;
    }

    public default TalonFX getRollerMotor() {
        return null;
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