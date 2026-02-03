package frc.robot.subsystems.kicker;

import static frc.robot.util.SparkUtil.*;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class KickerIOSpark implements KickerIO {
    private static SparkMax rollerMotor = new SparkMax(Constants.KickerConstants.rollerMotorID, MotorType.kBrushless);

  public KickerIOSpark() {

  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  public void stopAllMotors() {
    rollerMotor.setVoltage(0.0);
  }

  public void config() {
    SparkMaxConfig rollerConfig = new SparkMaxConfig();

    rollerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(20);

    tryUntilOk(
        rollerMotor,
        5,
        () -> rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.rollerConnected = rollerMotor.getLastError() == REVLibError.kOk;
    inputs.rollerMotorCurrentAmps = rollerMotor.getOutputCurrent();
    inputs.rollerMotorVoltageVolts = rollerMotor.getBusVoltage();
    inputs.rollerMotorSpeedRpm = rollerMotor.getEncoder().getVelocity();
  }

}
