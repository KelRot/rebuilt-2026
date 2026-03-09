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

  private static SparkMax spinnerMotor, topRollerMotor;

  /** Creates a new ExampleSubsystem. */
  public IndexIOSpark() {

    spinnerMotor = new SparkMax(Constants.IndexConstants.spinnerMotorID, MotorType.kBrushless);
    topRollerMotor = new SparkMax(Constants.IndexConstants.topRollerMotorID, MotorType.kBrushless);
    config();
  }

  @Override
  public void setVoltage(double spinVolts, double topVolts) {
    spinnerMotor.setVoltage(spinVolts);
    topRollerMotor.setVoltage(topVolts);
  }

  public void stopAllMotors() {
    spinnerMotor.setVoltage(0.0);
    topRollerMotor.setVoltage(0);
  }

  public void config() {
    SparkMaxConfig spinnerConfig = new SparkMaxConfig();

    spinnerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    tryUntilOk(
        spinnerMotor,
        2,
        () -> spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    tryUntilOk(
        topRollerMotor,
        2,
        () -> topRollerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(IndexIOInputs inputs) {
    inputs.spinnerConnected = spinnerMotor.getLastError() == REVLibError.kOk;
    inputs.spinnerMotorCurrentAmps = spinnerMotor.getOutputCurrent();
    inputs.spinnerMotorVoltageVolts = spinnerMotor.getBusVoltage();
    inputs.spinnerMotorSpeedRpm = spinnerMotor.getEncoder().getVelocity();
  }

}