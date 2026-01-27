package frc.robot.subsystems.index;

import static frc.robot.util.SparkUtil.*;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class IndexIOSpark implements IndexIO {

  private static SparkMax spinnerMotor = new SparkMax(Constants.IndexConstants.spinnerMotorID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public IndexIOSpark() {

  }

  @Override
  public void setSpinnerVoltage(double volts) {
    spinnerMotor.setVoltage(volts);
  }

  public void stopAllMotors() {
    spinnerMotor.setVoltage(0.0);
  }

  public void config() {
    SparkMaxConfig spinnerConfig = new SparkMaxConfig();

    spinnerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(20);

    tryUntilOk(
        spinnerMotor,
        5,
        () -> spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(IndexIOInputs inputs) {
    inputs.spinnerConnected = spinnerMotor.getLastError() == REVLibError.kOk;
    inputs.spinnerMotorCurrentAmps = spinnerMotor.getOutputCurrent();
    inputs.spinnerMotorVoltageVolts = spinnerMotor.getBusVoltage();
    inputs.spinnerMotorSpeedRpm = spinnerMotor.getEncoder().getVelocity();
  }

}