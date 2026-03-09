package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkTunablePID;
import frc.robot.util.SparkTunablePID.DriveType;

public class IntakeIOSpark implements IntakeIO {

  private static SparkMax openerMotor = new SparkMax(Constants.IntakeConstants.openerMotorID, MotorType.kBrushless);
  private static SparkMax secondOpenerMotor = new SparkMax(Constants.IntakeConstants.secondOpenerMotorID, MotorType.kBrushless);
  private static TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.rollerMotorID);

  /** Creates a new ExampleSubsystem. */
  public IntakeIOSpark() {
    openerMotor.getEncoder().setPosition(37);
    config();
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
     openerMotor.getEncoder().setPosition(37);
  }

  public void stopAllMotors() {
    rollerMotor.setVoltage(0.0);
    openerMotor.setVoltage(0.0);
  }

  public boolean isOpenerAtSetpoint() {
    boolean isAtSetpoint = openerMotor.getClosedLoopController().isAtSetpoint();
    return isAtSetpoint && Math.abs(openerMotor.getEncoder().getPosition() - openerMotor.getClosedLoopController().getSetpoint()) < 2.0;
  }

  public TalonFX getRollerMotor(){
    return rollerMotor;
  }

  public void config() {
    SparkMaxConfig openerConfig = new SparkMaxConfig();
    rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    openerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    openerConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(500).maxAcceleration(1500).allowedProfileError(2);
    openerConfig.softLimit.forwardSoftLimit(20).forwardSoftLimitEnabled(true).reverseSoftLimit(20)
        .reverseSoftLimitEnabled(true);
    openerConfig.encoder.positionConversionFactor(1.0 / Constants.IntakeConstants.openerGearRatio * 360.0);

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
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected = rollerMotor.isConnected();
    inputs.secondOpenerConnected = secondOpenerMotor.getLastError() == REVLibError.kOk;
    inputs.openerConnected = openerMotor.getLastError() == REVLibError.kOk;
    inputs.rollerMotorCurrentAmps = rollerMotor.getMotorStallCurrent().getValueAsDouble();
    inputs.rollerMotorVoltageVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerMotorSpeedRpm = rollerMotor.getVelocity().getValueAsDouble() * 60;
    inputs.openerMotorVelocityRPS = openerMotor.getEncoder().getVelocity() / 60;
    inputs.openerMotorCurrentAmps = openerMotor.getOutputCurrent();
    inputs.openerMotorVoltageVolts = openerMotor.getBusVoltage();
    inputs.IntakePosition = openerMotor.getEncoder().getPosition();
    inputs.isIntakeOpen = inputs.IntakePosition > Constants.IntakeConstants.intakeOpenPosition;
  }

}
