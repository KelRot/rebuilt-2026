package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

  public enum SystemState {
    IDLE,
    ENABLED,
    MANUAL,
    TESTING
  }

  private SystemState systemState = SystemState.IDLE;
  private double manualVoltage = 0.0;

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs =
      new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void requestState(SystemState wantedState) {
    systemState = wantedState;
  }

  public void setManualVoltage(double volts) {
    manualVoltage = volts;
    systemState = SystemState.MANUAL;
  }

  public void stop() {
    systemState = SystemState.IDLE;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (systemState) {
      case ENABLED:
        io.setKickerVoltage(Constants.KickerConstants.defaultKickerVoltage);
        break;

      case MANUAL:
        io.setKickerVoltage(manualVoltage);
        break;

      case IDLE:
      default:
        io.setKickerVoltage(0.0);
        break;
      case TESTING:
        io.setKickerVoltage(1);
        break;
    }

    Logger.recordOutput("Kicker/SystemState", systemState.toString());
    Logger.processInputs("Kicker", inputs);
  }
}
