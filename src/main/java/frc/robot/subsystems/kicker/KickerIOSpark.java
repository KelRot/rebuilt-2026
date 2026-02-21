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
    private static SparkMax kickerMotor = new SparkMax(Constants.KickerConstants.kickerMotorID, MotorType.kBrushless);

  public KickerIOSpark() {

  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotor.setVoltage(volts);
  }

  public void stopAllMotors() {
    kickerMotor.setVoltage(0.0);
  }

  public void config() {
    SparkMaxConfig kickerConfig = new SparkMaxConfig();

    kickerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    tryUntilOk(
        kickerMotor,
        5,
        () -> kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerConnected = kickerMotor.getLastError() == REVLibError.kOk;
    inputs.kickerMotorCurrentAmps = kickerMotor.getOutputCurrent();
    inputs.kickerMotorVoltageVolts = kickerMotor.getBusVoltage();
    inputs.kickerMotorSpeedRpm = kickerMotor.getEncoder().getVelocity();
  }

}
