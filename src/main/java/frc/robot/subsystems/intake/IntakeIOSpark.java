package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class IntakeIOSpark implements IntakeIO {

  private static SparkMax openerMotor = new SparkMax(Constants.IntakeConstants.openerMotorID, MotorType.kBrushless);
  private static SparkMax secondOpenerMotor = new SparkMax(Constants.IntakeConstants.secondOpenerMotorID, MotorType.kBrushless);
  private static SparkMax rollerMotor = new SparkMax(Constants.IntakeConstants.rollerMotorID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public IntakeIOSpark() {

  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void setOpenerSetPoint(double setPoint) {
    openerMotor.getClosedLoopController().setSetpoint(setPoint, ControlType.kMAXMotionPositionControl);
  }

  public void setOpenerVoltage(double volts) {
    openerMotor.setVoltage(volts);
  }

  public void zeroEncoder() {
     openerMotor.getEncoder().setPosition(0);
  }

  public void stopAllMotors() {
    rollerMotor.setVoltage(0.0);
    openerMotor.setVoltage(0.0);
  }

  public boolean isOpenerAtSetpoint() {
    boolean isAtSetpoint = openerMotor.getClosedLoopController().isAtSetpoint();
    return isAtSetpoint && Math.abs(openerMotor.getEncoder().getPosition() - openerMotor.getClosedLoopController().getSetpoint()) < 2.0;
  }

  public void config() {
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    SparkMaxConfig openerConfig = new SparkMaxConfig();

    rollerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    openerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    openerConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(500).maxAcceleration(1500).allowedProfileError(2);
    openerConfig.softLimit.forwardSoftLimit(20).forwardSoftLimitEnabled(true).reverseSoftLimit(20)
        .reverseSoftLimitEnabled(true);
    openerConfig.encoder.positionConversionFactor(1.0 / Constants.IntakeConstants.openerGearRatio * 360.0);

    tryUntilOk(
        rollerMotor,
        5,
        () -> rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    tryUntilOk(
        openerMotor,
        5,
        () -> openerMotor.configure(openerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    openerConfig.follow(openerMotor, true);
    tryUntilOk(
        secondOpenerMotor,
        5,
        () -> secondOpenerMotor.configure(openerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected = rollerMotor.getLastError() == REVLibError.kOk;
    inputs.secondOpenerConnected = secondOpenerMotor.getLastError() == REVLibError.kOk;
    inputs.openerConnected = openerMotor.getLastError() == REVLibError.kOk;
    inputs.rollerMotorCurrentAmps = rollerMotor.getOutputCurrent();
    inputs.rollerMotorVoltageVolts = rollerMotor.getBusVoltage();
    inputs.rollerMotorSpeedRpm = rollerMotor.getEncoder().getVelocity();
    inputs.openerMotorCurrentAmps = openerMotor.getOutputCurrent();
    inputs.openerMotorVoltageVolts = openerMotor.getBusVoltage();
    inputs.IntakePosition = openerMotor.getEncoder().getPosition();
    inputs.isIntakeOpen = inputs.IntakePosition > Constants.IntakeConstants.intakeOpenPosition;
  }

}
