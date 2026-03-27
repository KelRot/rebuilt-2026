package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class IntakeIOHardware implements IntakeIO {
  private SparkMax openerMotor = new SparkMax(Constants.IntakeConstants.openerMotorID, MotorType.kBrushless);
  private SparkMax secondOpenerMotor = new SparkMax(Constants.IntakeConstants.secondOpenerMotorID,
      MotorType.kBrushless);
  private TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.rollerMotorID);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public IntakeIOHardware() {
    openerMotor.getEncoder().setPosition(-37);
    rollerMotor.optimizeBusUtilization();
    config();
    setOpenerSetPoint(-37);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void setRollerRPM(double rpm) {
    double rps = rpm / 60.0;
    rollerMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void setOpenerSetPoint(double setPoint) {
    openerMotor.getClosedLoopController().setSetpoint(setPoint, ControlType.kPosition);
  }

  public TalonFX getRollerMotor() {
    return this.rollerMotor;
  }

  public SparkBase getLeadOpenerMotor() {
    return this.openerMotor;
  }

  public void setOpenerVoltage(double volts) {
    openerMotor.setVoltage(volts);
  }

  public void zeroEncoder() {
    openerMotor.getEncoder().setPosition(-37);
  }

  public void stopAllMotors() {
    rollerMotor.setVoltage(0.0);
    openerMotor.setVoltage(0.0);
  }

  public boolean isOpenerAtSetpoint() {
    boolean isAtSetpoint = openerMotor.getClosedLoopController().isAtSetpoint();
    return isAtSetpoint;
  }

  public void setBrake(boolean brake) {
    SparkMaxConfig conf = new SparkMaxConfig();
    if (brake) {
      conf.idleMode(IdleMode.kBrake);
    } else {
      conf.idleMode(IdleMode.kCoast);
    }
    openerMotor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    secondOpenerMotor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void config() {
    // --- SparkMax Opener Config ---
    SparkMaxConfig openerConfig = new SparkMaxConfig();
    openerConfig.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    openerConfig.closedLoop.pid(0, 0, 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder).allowedClosedLoopError(4,
        ClosedLoopSlot.kSlot0);
    openerConfig.encoder.positionConversionFactor(1.0 / Constants.IntakeConstants.openerGearRatio * 360.0);
    tryUntilOk(openerMotor, 5,
        () -> openerMotor.configure(openerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkMaxConfig opener2Config = new SparkMaxConfig();
    opener2Config.voltageCompensation(12).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    opener2Config.follow(3, true);
    tryUntilOk(secondOpenerMotor, 5,
        () -> secondOpenerMotor.configure(opener2Config, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));
    // --- TalonFX Roller Config ---
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    rollerConfig.Audio.BeepOnBoot = true;
    rollerConfig.Feedback.RotorToSensorRatio = 1.0;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 120;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerConfig.Slot0.kP = 0.001;
    rollerConfig.Slot0.kI = 0.0;
    rollerConfig.Slot0.kD = 0.0;
    rollerConfig.Slot0.kV = 1 / 509.3 / 60;

    rollerMotor.getConfigurator().apply(rollerConfig);

  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected = rollerMotor.isConnected();
    inputs.secondOpenerConnected = secondOpenerMotor.getLastError() == REVLibError.kOk;
    inputs.openerConnected = openerMotor.getLastError() == REVLibError.kOk;
    inputs.rollerMotorCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerMotorVoltageVolts = rollerMotor.getMotorVoltage(true).getValueAsDouble();
    inputs.rollerMotorSpeedRpm = rollerMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.openerMotorVelocityRPS = openerMotor.getEncoder().getVelocity() / 60.0;
    inputs.openerMotorCurrentAmps = openerMotor.getOutputCurrent();
    inputs.openerMotorVoltageVolts = openerMotor.getBusVoltage();
    inputs.IntakePosition = openerMotor.getEncoder().getPosition();
    inputs.isIntakeOpen = inputs.IntakePosition + 3 > Constants.IntakeConstants.intakeOpenPosition;
  }
}