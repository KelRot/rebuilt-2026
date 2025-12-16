package frc.robot.util.motors.controllers;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.motors.MotorIO;

public class SparkBaseController implements MotorIO {

    private SparkBase spark;
    private SparkBaseConfig sparkConfig;

    /**
     * Setups the motor..
     *
     * @param motor The {@link Motor} dataclass containing motor configuration
     *              details
     * @return the {@code MotorConfigurator} instance for chaining
     * @throws IllegalArgumentException if the motor type is unsupported
     */
    public SparkBaseController(Motor motor) {
        createSparkMotor(motor.controllerType(), motor.canID());
    }

    void createSparkMotor(ControllerType motorType, int canID) {
        switch (motorType) {
            case SPARKMAX:
                spark = new SparkMax(canID, MotorType.kBrushless);
                sparkConfig = new SparkMaxConfig();
                break;
            case SPARKFLEX:
                spark = new SparkFlex(canID, MotorType.kBrushless);
                sparkConfig = new SparkFlexConfig();
                break;
            default:
                throw new IllegalArgumentException("Invalid SparkBase Motor type");
        }

    }

    public void setNeutralMode(boolean b) {
        sparkConfig.idleMode(b ? IdleMode.kBrake : IdleMode.kCoast);
        spark.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setClosedLoopRampRate(int closedLoopRampRate) {
        sparkConfig.closedLoopRampRate(closedLoopRampRate);
        spark.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setOpenLoopRampRate(int openLoopRampRate) {
        sparkConfig.openLoopRampRate(openLoopRampRate);
        spark.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}
