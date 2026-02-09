package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  public enum IndexSystemState {
    INDEXING_BALL,
    PASSIVE_MODE,
    OFF
  }

  private IndexSystemState systemState = IndexSystemState.OFF;

  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  public Index(IndexIO io) {
    this.io = io;
  }

  private void updateState() {
    if (inputs.spinnerMotorCurrentAmps > Constants.IndexConstants.spinnerAmpsLimit) {
      systemState = IndexSystemState.INDEXING_BALL;
    } 
    else {
      systemState = IndexSystemState.PASSIVE_MODE;
    }
  }
  private void runStateMachine() {
    double voltage = 0.0;

    switch (systemState) {
      case INDEXING_BALL:
        voltage = Constants.IndexConstants.INDEXING_VOLTAGE;
        break;

      case PASSIVE_MODE:
        voltage = Constants.IndexConstants.PASSIVE_MODE_VOLTAGE;
        break;

      case OFF:
      default:
        voltage = 0.0;
        break;
    }

    io.setSpinnerVoltage(voltage);
  }

  public void setState(IndexSystemState state) {
    this.systemState = state;
  }

  public Command setStateCommand(IndexSystemState state) {
    return this.runOnce(() -> setState(state));
  }

  public Command stopCommand() {
    return setStateCommand(IndexSystemState.OFF);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateState();
    runStateMachine();

    Logger.processInputs("Index", inputs);
    Logger.recordOutput("Index/SystemState", systemState.toString());
  }

  @Override
  public void simulationPeriodic() {}
}