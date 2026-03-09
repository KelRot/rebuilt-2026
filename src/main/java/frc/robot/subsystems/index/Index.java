package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  /* ---------------- State ---------------- */

  public enum SystemState {
    IDLE,
    INDEXING,
    PASSIVE,
    OUTTAKING,
    TESTING
  }

  public SystemState systemState = SystemState.IDLE;

  private final IndexIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  public Index(IndexIO io) {
    this.io = io;
  }


  public void requestState(SystemState wantedState) {
    Commands.print("dasdsa");
    this.systemState = wantedState;
  }

  public void stop() {
    requestState(SystemState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (this.systemState) {

      case INDEXING:
        io.setVoltage(Constants.IndexConstants.INDEXING_VOLTAGE, Constants.IndexConstants.INDEXING_TOPROLLER_VOLTAGE);
        break;

      case PASSIVE:
        io.setVoltage(Constants.IndexConstants.PASSIVE_MODE_VOLTAGE, Constants.IndexConstants.PASSIVE_MODE_TOPROLLER_VOLTAGE);
        break;

      case IDLE:
        io.setVoltage(0.0, 0);
        io.stopAllMotors();
        break;

      case OUTTAKING:
          io.setVoltage(Constants.IndexConstants.OUTTAKING_VOLTAGE, Constants.IndexConstants.OUTTAKING_TOPROLLER_VOLTAGE);
          break;

      case TESTING:
          io.setVoltage(1, 1);
          break;
    }

    Logger.recordOutput("Index/SystemState", systemState.toString());
    Logger.processInputs("Index", inputs);
    SmartDashboard.putNumber("Intake voltage", inputs.intakeVoltage);
  }
}
