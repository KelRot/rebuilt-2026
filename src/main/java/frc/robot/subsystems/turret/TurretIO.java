package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public boolean motorConnected = false;
        public double positionRads = 0.0; //Relative position
        public double absolutePositionRads = 0.0; //Absolute position from absolute encoder
        public double velocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public boolean hallEffectTriggered = false;
    }

    /* Update inputs from hardware */
    public default void updateInputs(TurretIOInputs inputs){}

    /* Set voltage to turret motor */
    public default void setVoltage(double voltage){}

    /* Set turret position */
    public default void setPosition(double setpoint){}

    /* Zero the encoder */
    public default void zeroEncoder(){}

    /* Stop the turret motor */
    public default void stop(){}

    /* Set idle mode */
    public default void setIdleMode(IdleMode mode){}



}