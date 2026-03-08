package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.TurretConstants;

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
import edu.wpi.first.wpilibj.Encoder;

public class TurretIOSpark implements TurretIO {
    //Hardware components
    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    private final DutyCycleEncoder absEncoder1;
    private final DutyCycleEncoder absEncoder2;

    private double targetPositionDeg;

    private final SparkClosedLoopController turretController;

    private final Debouncer turretDebouncer = new Debouncer(0.5);

    public TurretIOSpark(){
        turretMotor = new SparkMax(TurretConstants.turretID, MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        absEncoder1 = new DutyCycleEncoder(TurretConstants.absEncoder1ID, 360.0, TurretConstants.abs1Offset);
        absEncoder2 = new DutyCycleEncoder(TurretConstants.absEncoder2ID, 360.0, TurretConstants.abs2Offset);
        turretController = turretMotor.getClosedLoopController();

        configure();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        sparkStickyFault = false;
        ifOk(turretMotor, turretEncoder::getPosition, (value)-> inputs.positionRads = value);
        ifOk(turretMotor, turretEncoder::getVelocity, (value)-> inputs.velocityRadsPerSec = value);
        ifOk(turretMotor, new DoubleSupplier[] {turretMotor::getAppliedOutput, turretMotor::getBusVoltage}, 
            (values)-> inputs.appliedVolts = values[0] * values[1]);
        ifOk(turretMotor, turretMotor::getOutputCurrent, (value)-> inputs.supplyCurrentAmps = value);
        inputs.motorConnected = turretDebouncer.calculate(!sparkStickyFault);
        inputs.absPositionTours1 = absEncoder1.get();
        inputs.absPositionTours2 = absEncoder2.get();
    }

    @Override
    public void setVoltage(double voltage){
        turretMotor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocity){
        turretController.setSetpoint(velocity, ControlType.kVelocity);
    }

    @Override
    public void setPosition(double setpoint){
        targetPositionDeg = setpoint*360;//CHECK LATER
        turretController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void setEncoder(double position){
        turretEncoder.setPosition(position);
    }

    @Override
    public void stop(){
        turretMotor.setVoltage(0.0);
    }

    private void configure(){
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(TurretConstants.kP, 0, TurretConstants.kD);

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(20)
            .apply(closedLoopConfig);
        turretConfig
            .encoder.positionConversionFactor(TurretConstants.positionConversionFactor)
            .velocityConversionFactor(TurretConstants.velocityConversionFactor);
        turretConfig
            .closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .cruiseVelocity(TurretConstants.cruiseVelocity)
            .maxAcceleration(TurretConstants.maxAcceleration)
            .allowedProfileError(2);
        turretConfig
            .softLimit
            .forwardSoftLimit(TurretConstants.maxAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.minAngle)
            .reverseSoftLimitEnabled(true);

        tryUntilOk(turretMotor, 5, ()->
        turretMotor.configure(turretConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
    @Override
        public boolean isAtSetpoint() {
            double angle = turretEncoder.getPosition()*360;
            return Math.abs(angle - targetPositionDeg) < 5;
}
}
