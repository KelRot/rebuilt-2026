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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSparkFlex extends SubsystemBase implements FlyWheelIO {

    private final SparkFlex masterMotor;
    private final SparkFlex followerMotor;
    private final RelativeEncoder elencoder;
    private final PIDController pid;

    private final SparkFlexConfig masterConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();

    public FlywheelSparkFlex() {

        pid = new PIDController(Constants.FlywheelConstants.kp,Constants.FlywheelConstants.kd,Constants.FlywheelConstants.ki);

        // Initialize motors
        masterMotor = new SparkFlex(
                Constants.FlywheelConstants.kMasterMotorId,
                MotorType.kBrushless
        );

        followerMotor = new SparkFlex(
                Constants.FlywheelConstants.kFollowerMotorId,
                MotorType.kBrushless
        
        );
        elencoder = masterMotor.getEncoder();
        configureMotors();}
        @Override
        public void updateInputs(FlywheelIOInputs inputs) {
            inputs.followerConnected = followerMotor.getLastError() == REVLibError.kOk;
            
            inputs.velocityRadsPerSec = elencoder.getVelocity() * (2 * Math.PI / 60);
            inputs.appliedVoltage = masterMotor.getAppliedOutput() * 12;
            inputs.outputCurrentAmps = masterMotor.getOutputCurrent();
            inputs.leadcurrentRpm = elencoder.getVelocity();

            inputs.masterConnected = masterMotor.getLastError() == REVLibError.kOk;
            inputs.followerSupplyCurrentAmps = followerMotor.getOutputCurrent();
            inputs.followercurrentRpm = followerMotor.getEncoder().getVelocity();

        }

    private void configureMotors() {
        masterConfig
                .voltageCompensation(0)
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast);

        followerConfig
                .follow(Constants.FlywheelConstants.kMasterMotorId)
                .voltageCompensation(0)
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kCoast);

        masterMotor.configure(
                masterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        followerMotor.configure(
                followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );
    }
    @Override
    public void stopAllMotors() {
        masterMotor.stopMotor();
        followerMotor.stopMotor();
    }
    @Override
    public void setVoltage(double volts) {
        masterMotor.setVoltage(volts);
    }
        @Override
    public void setRpm(double targetRpm) {
        double targetRadsPerSec = targetRpm * (2 * Math.PI / 60);
        double currentRadsPerSec = elencoder.getVelocity() * (2 * Math.PI / 60);
        double outputVolts = pid.calculate(currentRadsPerSec, targetRadsPerSec);
        setVoltage(outputVolts);
        }
}
