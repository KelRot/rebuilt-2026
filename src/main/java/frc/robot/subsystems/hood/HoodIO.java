package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs{
        public boolean motorConnected = false;
        public double positionRads = 0.0; //Relative position
        public double absPositionTours1 = 0.0; //Absolute position from absolute encoder 1
        public double absPositionTours2 = 0.0; //Absolute position from absolute encoder 2
        public double velocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    /* Update inputs from hardware */
    public default void updateInputs(HoodIOInputs inputs){}

    /* Set voltage to hood motor */
    public default void setVoltage(double voltage){}

    /* Set hood position */
    public default void setPosition(double setpoint){}

    /* Set the encoder */
    public default void setEncoder(double position){}

    /* Stop the hood motor */
    public default void stop(){}

    /* Set idle mode */
    public default void setIdleMode(IdleMode mode){}



}