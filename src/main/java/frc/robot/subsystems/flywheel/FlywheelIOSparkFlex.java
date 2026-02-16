package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class FlywheelIOSparkFlex implements FlywheelIO {

    private final SparkFlex leadMotor;
    private final SparkFlex followerMotor;
    private final RelativeEncoder leadEncoder;
    private final PIDController pid;

    private final SparkFlexConfig leadConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();

    public FlywheelIOSparkFlex() {

        pid = new PIDController(
                Constants.FlywheelConstants.kp,
                Constants.FlywheelConstants.ki,
                Constants.FlywheelConstants.kd);

        leadMotor = new SparkFlex(
                Constants.FlywheelConstants.kMasterMotorId,
                MotorType.kBrushless);

        followerMotor = new SparkFlex(
                Constants.FlywheelConstants.kFollowerMotorId,
                MotorType.kBrushless);

        leadEncoder = leadMotor.getEncoder();

        configureMotors();
    }

    /* ---------------- IO ---------------- */

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {

        inputs.leadConnected = leadMotor.getLastError() == REVLibError.kOk;

        inputs.followerConnected = followerMotor.getLastError() == REVLibError.kOk;

        // Lead motor
        inputs.leadVelocityPerSec = leadEncoder.getVelocity() / 60.0;

        inputs.leadCurrentRpm = leadEncoder.getVelocity();

        inputs.leadAppliedVoltage = leadMotor.getAppliedOutput() * 12.0;

        inputs.leadCurrentAmps = leadMotor.getOutputCurrent();

        // Follower motor
        inputs.followerVelocityPerSec = followerMotor.getEncoder().getVelocity() / 60.0;

        inputs.followerCurrentRpm = followerMotor.getEncoder().getVelocity();

        inputs.followerAppliedVoltage = followerMotor.getAppliedOutput() * 12.0;

        inputs.followerCurrentAmps = followerMotor.getOutputCurrent();
    }

    /* ---------------- Motor config ---------------- */

    private void configureMotors() {

        leadConfig
                .voltageCompensation(12.0)
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast);

        followerConfig
                .follow(leadMotor, true)
                .voltageCompensation(12.0)
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast);

        leadMotor.configure(
                leadConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        followerMotor.configure(
                followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /* ---------------- Outputs ---------------- */

    @Override
    public void stop() {
        leadMotor.stopMotor();
    }

    @Override
    public void setAppliedVoltage(double volts) {
        leadMotor.setVoltage(volts);
    }

    @Override
    public void setRpm(double targetRpm) {
        double currentRPM = leadEncoder.getVelocity();
        double outputVolts = pid.calculate(currentRPM, targetRpm);
        setAppliedVoltage(outputVolts);
    }

    public boolean isAtSetpoint() {
        return pid.atSetpoint();
    }
}
