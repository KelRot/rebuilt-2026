package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

  public enum KickerMode {
    KICKER_ENABLED,
    KICKER_DISABLED,
  }

  private KickerMode systemState = KickerMode.KICKER_DISABLED;

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  public void setState(KickerMode mode) {
    this.systemState = mode;
  }

  public Command setRollerVoltageCommand(double volts) {
    return this.runEnd(() -> setRollerVoltage(volts), () -> setRollerVoltage(0.0));
  }

  public Command setStateCommand(KickerMode mode) {
    return this.runOnce(() -> systemState = mode);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    switch (systemState) {

      case KICKER_DISABLED:
        setRollerVoltage(0.0);
        break;

      case KICKER_ENABLED:
        setRollerVoltage(Constants.KickerConstants.defaultRollerVoltage);
        break;
    }
    Logger.recordOutput("Kicker/SystemState", systemState.toString());
    Logger.processInputs("Kicker", inputs);
  }

  @Override
  public void simulationPeriodic() {
  }
}
