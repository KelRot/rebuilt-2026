package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;

public class HoodIOSpark implements HoodIO {

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
    controller.setSetpoint(positionDeg, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void stop() {
    motor.disable();
  }

  private void configure() {
    ClosedLoopConfig closedLoop = new ClosedLoopConfig();
    closedLoop.pid(Constants.HoodConstants.kP, 0.0, Constants.HoodConstants.kD);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .voltageCompensation(12.0)
        .smartCurrentLimit(20)
        .apply(closedLoop);

    config.encoder
        .positionConversionFactor(Constants.HoodConstants.positionConversionFactorDeg);

    config.closedLoop.maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(Constants.HoodConstants.cruiseVelocityDegPerSec)
        .maxAcceleration(Constants.HoodConstants.maxAccelerationDegPerSec2)
        .allowedProfileError(1.0);

    config.softLimit
        .forwardSoftLimit(Constants.HoodConstants.maxAngleDeg)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.HoodConstants.minAngleDeg)
        .reverseSoftLimitEnabled(true);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }
}
