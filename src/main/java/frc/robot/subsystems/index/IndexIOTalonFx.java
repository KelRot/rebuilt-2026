package frc.robot.subsystems.index;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexIOTalonFx implements IndexIO {

  private final TalonFX spinnerMotor;

  public IndexIOTalonFx() {

    spinnerMotor = new TalonFX(1, CANBus.roboRIO());

    TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true));

    spinnerMotor.getConfigurator().apply(config);
  }

  @Override
  public void setSpinnerVoltage(double volts) {
    spinnerMotor.setVoltage(volts);
  }
  @Override
  public void stopAllMotors() {
    spinnerMotor.setVoltage(0.0);
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.spinnerConnected = spinnerMotor.isAlive();
    inputs.spinnerMotorCurrentAmps =
        spinnerMotor.getStatorCurrent().getValueAsDouble();
    inputs.spinnerMotorVoltageVolts =
        spinnerMotor.getMotorVoltage().getValueAsDouble();
    inputs.spinnerMotorSpeedRpm =
        spinnerMotor.getVelocity().getValueAsDouble() * 60.0;
  }
}