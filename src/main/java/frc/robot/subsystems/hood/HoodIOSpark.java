package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;

public class HoodIOSpark implements HoodIO {

  private double targetPositionDeg;

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public HoodIOSpark() {
    motor = new SparkMax(Constants.HoodConstants.hoodID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
    configure();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, value -> inputs.positionDeg = value);
    inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(double positionDeg) {
    targetPositionDeg = positionDeg;
    controller.setSetpoint(positionDeg, ControlType.kPosition);
  }

  @Override
  public void stop() {
    motor.disable();
  }

  public void setAppliedVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  private void configure() {
    ClosedLoopConfig closedLoop = new ClosedLoopConfig();
    closedLoop.pid(Constants.HoodConstants.kP, 0.0, Constants.HoodConstants.kD).allowedClosedLoopError(1,
        ClosedLoopSlot.kSlot0);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(25)
        .inverted(true)
        .apply(closedLoop);

    config.encoder
        .positionConversionFactor(10);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters));

  }

  @Override
  public void setEncoder(double deg) {
    motor.getEncoder().setPosition(deg);
  }

  @Override
  public SparkBase getMotor() {
    return motor;
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(targetPositionDeg - encoder.getPosition()) < 5.0;
  }

}
