package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class IntakeIOSpark implements IntakeIO {

  private static SparkMax rollerMotor = new SparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public IntakeIOSpark() {
    config();
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  public void setRollerRPM(double rpm) {
    rollerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rpm / Constants.IntakeConstants.RotPerVolt);
 
  }

  @Override
  public SparkBase getRollerMotor() {
      return rollerMotor;
  }
  public void config() {
    SparkMaxConfig rollerConfig = new SparkMaxConfig();

    rollerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    tryUntilOk(
        rollerMotor,
        5,
        () -> rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected = rollerMotor.getLastError() == REVLibError.kOk;
    inputs.rollerMotorCurrentAmps = rollerMotor.getOutputCurrent();
    inputs.rollerMotorVoltageVolts = rollerMotor.getBusVoltage();
    inputs.rollerMotorSpeedRpm = rollerMotor.getEncoder().getVelocity();
    inputs.isIntakeOpen = inputs.IntakePosition > Constants.IntakeConstants.intakeOpenPosition;
  }

}
