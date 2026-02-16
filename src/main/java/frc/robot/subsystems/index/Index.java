package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  /* ---------------- State ---------------- */

  public enum SystemState {
    IDLE,
    INDEXING,
    PASSIVE
  }

  private SystemState systemState = SystemState.IDLE;

  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  public Index(IndexIO io) {
    this.io = io;
  }


  public void requestState(SystemState wantedState) {
    systemState = wantedState;
  }

  public void stop() {
    requestState(SystemState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (systemState) {

      case INDEXING:
        io.setSpinnerVoltage(Constants.IndexConstants.INDEXING_VOLTAGE);
        break;

      case PASSIVE:
        io.setSpinnerVoltage(Constants.IndexConstants.PASSIVE_MODE_VOLTAGE);
        break;

      case IDLE:
      default:
        io.setSpinnerVoltage(0.0);
        break;
    }

    Logger.recordOutput("Index/SystemState", systemState.toString());
    Logger.processInputs("Index", inputs);
  }
}
