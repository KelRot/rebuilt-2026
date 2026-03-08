package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public boolean motorConnected = false;
        public double positionRads = 0.0; //Relative position
        public double absPositionTours1 = 0.0; //Absolute position from absolute encoder 1
        public double absPositionTours2 = 0.0; //Absolute position from absolute encoder 2
        public double velocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    /* Update inputs from hardware */
    public default void updateInputs(TurretIOInputs inputs){}

    /* Set voltage to turret motor */
    public default void setVoltage(double voltage){}

    /* Set turret velocity */
    public default void setVelocity(double velocity){}

    /* Set turret position */
    public default void setPosition(double setpoint){}

    /* Set the encoder */
    public default void setEncoder(double position){}

    /* Stop the turret motor */
    public default void stop(){}

    public default boolean isAtSetpoint(){
        return false;
    }



}