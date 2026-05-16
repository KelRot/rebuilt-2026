// ModuleIOSpark.java

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.util.BrakeController;

import java.util.Queue;

public class ModuleIOSpark implements ModuleIO {

    private final Rotation2d zeroRotation;

    private final SparkBase driveSpark;
    private final SparkBase turnSpark;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final AnalogInput turnAbsEncoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final Debouncer driveConnectedDebounce =
            new Debouncer(0.5);

    private final Debouncer turnConnectedDebounce =
            new Debouncer(0.5);

    public ModuleIOSpark(int module) {

        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();
        };

        driveSpark =
                new SparkFlex(
                        switch (module) {
                            case 0 -> frontLeftDriveCanId;
                            case 1 -> frontRightDriveCanId;
                            case 2 -> backLeftDriveCanId;
                            case 3 -> backRightDriveCanId;
                            default -> 0;
                        },
                        MotorType.kBrushless);

        turnSpark =
                new SparkMax(
                        switch (module) {
                            case 0 -> frontLeftTurnCanId;
                            case 1 -> frontRightTurnCanId;
                            case 2 -> backLeftTurnCanId;
                            case 3 -> backRightTurnCanId;
                            default -> 0;
                        },
                        MotorType.kBrushless);

        turnAbsEncoder =
                new AnalogInput(
                        switch (module) {
                            case 0 -> frontLeftTurnAbsId;
                            case 1 -> frontRightTurnAbsId;
                            case 2 -> backLeftTurnAbsId;
                            case 3 -> backRightTurnAbsId;
                            default -> 0;
                        });

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // DRIVE CONFIG

        var driveConfig = new SparkFlexConfig();

        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);

        driveConfig.encoder
                .positionConversionFactor(
                        driveEncoderPositionFactor)
                .velocityConversionFactor(
                        driveEncoderVelocityFactor)
                .uvwMeasurementPeriod(20)
                .uvwAverageDepth(2);

        driveConfig.closedLoop
                .feedbackSensor(
                        FeedbackSensor.kPrimaryEncoder)
                .pid(driveKp, 0.0, driveKd);

        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(false)
                .appliedOutputPeriodMs(50)
                .busVoltagePeriodMs(50)
                .outputCurrentPeriodMs(50);

        tryUntilOk(
                driveSpark,
                5,
                () ->
                        driveSpark.configure(
                                driveConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        tryUntilOk(
                driveSpark,
                5,
                () -> driveEncoder.setPosition(0.0));

        // TURN CONFIG

        var turnConfig = new SparkMaxConfig();

        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);

        turnConfig.encoder
                .positionConversionFactor(
                        turnEncoderPositionFactor)
                .velocityConversionFactor(
                        turnEncoderVelocityFactor)
                .uvwMeasurementPeriod(20)
                .uvwAverageDepth(2);

        turnConfig.closedLoop
                .feedbackSensor(
                        FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(
                        turnPIDMinInput,
                        turnPIDMaxInput)
                .pid(turnKp, 0.0, turnKd);

        turnConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(false)
                .appliedOutputPeriodMs(50)
                .busVoltagePeriodMs(50)
                .outputCurrentPeriodMs(50);

        tryUntilOk(
                turnSpark,
                5,
                () ->
                        turnSpark.configure(
                                turnConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        tryUntilOk(
                turnSpark,
                5,
                () ->
                        turnEncoder.setPosition(
                                (turnAbsEncoder.getAverageVoltage()
                                                / RobotController.getVoltage5V())
                                        * 2.0
                                        * Math.PI));

        BrakeController.register(driveSpark);
        BrakeController.register(turnSpark);

        timestampQueue =
                SparkOdometryThread.getInstance()
                        .makeTimestampQueue();

        drivePositionQueue =
                SparkOdometryThread.getInstance()
                        .registerSignal(
                                driveSpark,
                                driveEncoder::getPosition);

        turnPositionQueue =
                SparkOdometryThread.getInstance()
                        .registerSignal(
                                turnSpark,
                                turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        sparkStickyFault = false;

        // DRIVE

        double drivePosition =
                driveEncoder.getPosition();

        double driveVelocity =
                driveEncoder.getVelocity();

        double driveApplied =
                driveSpark.getAppliedOutput();

        double driveBus =
                driveSpark.getBusVoltage();

        double driveCurrent =
                driveSpark.getOutputCurrent();

        inputs.drivePositionRad = drivePosition;

        inputs.driveVelocityRadPerSec =
                driveVelocity;

        inputs.driveAppliedVolts =
                driveApplied * driveBus;

        inputs.driveCurrentAmps =
                driveCurrent;

        inputs.driveConnected =
                driveConnectedDebounce.calculate(
                        !sparkStickyFault);

        // TURN

        double turnPosition =
                turnEncoder.getPosition();

        double turnVelocity =
                turnEncoder.getVelocity();

        double turnApplied =
                turnSpark.getAppliedOutput();

        double turnBus =
                turnSpark.getBusVoltage();

        double turnCurrent =
                turnSpark.getOutputCurrent();

        inputs.turnPosition =
                new Rotation2d(turnPosition)
                        .minus(zeroRotation);

        inputs.turnVelocityRadPerSec =
                turnVelocity;

        inputs.turnAppliedVolts =
                turnApplied * turnBus;

        inputs.turnCurrentAmps =
                turnCurrent;

        inputs.turnConnected =
                turnConnectedDebounce.calculate(
                        !sparkStickyFault);

        // ODOMETRY

        inputs.odometryTimestamps =
                timestampQueue.stream()
                        .mapToDouble(Double::doubleValue)
                        .toArray();

        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                        .mapToDouble(Double::doubleValue)
                        .toArray();

        Double[] turnArray =
                turnPositionQueue.toArray(
                        new Double[0]);

        Rotation2d[] rotations =
                new Rotation2d[turnArray.length];

        for (int i = 0; i < turnArray.length; i++) {

            rotations[i] =
                    new Rotation2d(turnArray[i])
                            .minus(zeroRotation);
        }

        inputs.odometryTurnPositions =
                rotations;

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {

        double ffVolts =
                driveKs * Math.signum(velocityRadPerSec)
                        + driveKv * velocityRadPerSec;

        driveController.setSetpoint(
                velocityRadPerSec,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {

        double setpoint =
                MathUtil.inputModulus(
                        rotation.plus(zeroRotation)
                                .getRadians(),
                        turnPIDMinInput,
                        turnPIDMaxInput);

        turnController.setSetpoint(
                setpoint,
                ControlType.kPosition);
    }
}