package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.kicker.KickerIO.KickerIOInputs;

public class Kicker extends SubsystemBase {

  public enum IndexMode {
        KICKER_ENABLED,
        KICKER_DISABLED,
    }   

  private IndexMode currentMode = IndexMode.KICKER_DISABLED;

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }


  private void checkMode() {
    if (inputs.rollerMotorCurrentAmps >= Constants.KickerConstants.rollerAmpsLimit) {
      currentMode = IndexMode.KICKER_ENABLED;
    } else {
      currentMode = IndexMode.KICKER_DISABLED;
    }
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  private void runStateMachine() {
  switch (currentMode) {

    case KICKER_DISABLED:
      setRollerVoltage(0.0);
      break;

    case KICKER_ENABLED:
      setRollerVoltage(Constants.KickerConstants.defaultRollerVoltage);
      break;
  }
}

  public Command setRollerVoltageCommand(double volts) {
    return this.runEnd(() -> setRollerVoltage(volts), () -> setRollerVoltage(0.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    checkMode();
    runStateMachine();
    Logger.processInputs("Kicker", inputs);
  }

  @Override
  public void simulationPeriodic() {
  }
}
