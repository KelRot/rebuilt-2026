package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.TurretConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.Debouncer;

public class TurretIOSpark implements TurretIO {

    private final SparkFlex turretMotor;
    private final RelativeEncoder turretEncoder;

    private double targetPositionDeg;

    private final SparkClosedLoopController turretController;

    private final TurretAngleCalculator angleCalculator = new TurretAngleCalculator();

    private final Debouncer turretDebouncer = new Debouncer(0.5);

    // Açı hesabını ve log'u her 5 döngüde bir yap (100ms)
    private int angleCounter = 0;

    public TurretIOSpark() {
        turretMotor = new SparkFlex(TurretConstants.turretID, MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        turretController = turretMotor.getClosedLoopController();

        configure();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(turretMotor, turretEncoder::getPosition, (value) -> inputs.positionRads = value);
        ifOk(turretMotor, turretEncoder::getVelocity, (value) -> inputs.velocityRadsPerSec = value);
        ifOk(turretMotor, new DoubleSupplier[] { turretMotor::getAppliedOutput, turretMotor::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(turretMotor, turretMotor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
        inputs.motorConnected = turretDebouncer.calculate(!sparkStickyFault);
        inputs.absPositionTours1 = angleCalculator.getSmallEncoder();
        inputs.absPositionTours2 = angleCalculator.getBigEncoder();

        // Ağır CRT hesabını ve SmartDashboard log'unu her 5 döngüde bir çalıştır
        if (angleCounter++ >= 5) {
            angleCounter = 0;
            angleCalculator.log();
        }
    }

    @Override
    public void setVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocity) {
        turretController.setSetpoint(velocity, ControlType.kVelocity);
    }

    @Override
    public void setPosition(double setpoint) {
        targetPositionDeg = setpoint;
        turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setEncoder(double position) {
        turretEncoder.setPosition(position);
    }

    @Override
    public void stop() {
        turretMotor.setVoltage(0.0);
    }

    public TurretAngleCalculator getAngleCalculator() {
        return angleCalculator;
    }

    private void configure() {
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(TurretConstants.kP, 0, TurretConstants.kD).allowedClosedLoopError(2, ClosedLoopSlot.kSlot0);

        SparkFlexConfig turretConfig = new SparkFlexConfig();
        turretConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(25)
                .apply(closedLoopConfig);
        turretConfig.encoder.positionConversionFactor(TurretConstants.positionConversionFactor)
                .velocityConversionFactor(TurretConstants.velocityConversionFactor);
        turretConfig.softLimit
                .forwardSoftLimit(TurretConstants.maxAngle)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(TurretConstants.minAngle)
                .reverseSoftLimitEnabled(true);

        tryUntilOk(turretMotor, 5, () -> turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    }

    @Override
    public SparkBase getMotor() {
        return turretMotor;
    }

    @Override
    public boolean isAtSetpoint() {
        double angle = turretEncoder.getPosition();
        return Math.abs(angle - targetPositionDeg) < 5;
    }
}