package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class HoodIOSpark implements HoodIO {
    //Hardware components
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final DutyCycleEncoder absEncoder1;
    private final DutyCycleEncoder absEncoder2;

    private IdleMode idleMode;

    private final SparkClosedLoopController hoodController;

    private final Debouncer hoodDebouncer = new Debouncer(0.5);

    public HoodIOSpark(){
        hoodMotor = new SparkMax(HoodConstants.hoodID, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        absEncoder1 = new DutyCycleEncoder(HoodConstants.absEncoder1ID);
        absEncoder2 = new DutyCycleEncoder(HoodConstants.absEncoder2ID);

        idleMode = IdleMode.kBrake;

        hoodController = hoodMotor.getClosedLoopController();

        configure();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs){
        sparkStickyFault = false;
        ifOk(hoodMotor, hoodEncoder::getPosition, (value)-> inputs.positionRads = value);
        ifOk(hoodMotor, hoodEncoder::getVelocity, (value)-> inputs.velocityRadsPerSec = value);
        ifOk(hoodMotor, new DoubleSupplier[] {hoodMotor::getAppliedOutput, hoodMotor::getBusVoltage}, 
            (values)-> inputs.appliedVolts = values[0] * values[1]);
        ifOk(hoodMotor, hoodMotor::getOutputCurrent, (value)-> inputs.supplyCurrentAmps = value);
        inputs.motorConnected = hoodDebouncer.calculate(!sparkStickyFault);
        inputs.absPositionTours1 = absEncoder1.get();
        inputs.absPositionTours2 = absEncoder2.get();
    }

    @Override
    public void setVoltage(double voltage){
        hoodMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double setpoint){
        hoodController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void setEncoder(double position){
        hoodEncoder.setPosition(position);
    }

    @Override
    public void stop(){
        hoodMotor.setVoltage(0.0);
    }

    @Override
    public void setIdleMode(IdleMode mode){
        idleMode = mode;
        configure();
    }

    private void configure(){
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(HoodConstants.kP, 0, HoodConstants.kD);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig
            .idleMode(idleMode)
            .voltageCompensation(12)
            .smartCurrentLimit(20)
            .apply(closedLoopConfig);
        hoodConfig
            .encoder.positionConversionFactor(HoodConstants.positionConversionFactor)
            .velocityConversionFactor(HoodConstants.velocityConversionFactor);
        hoodConfig
            .closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .cruiseVelocity(HoodConstants.cruiseVelocity)
            .maxAcceleration(HoodConstants.maxAcceleration)
            .allowedProfileError(2);
        hoodConfig
            .softLimit
            .forwardSoftLimit(HoodConstants.maxAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(HoodConstants.minAngle)
            .reverseSoftLimitEnabled(true);

        tryUntilOk(hoodMotor, 5, ()->
        hoodMotor.configure(hoodConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

}