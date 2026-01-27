package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

    public enum IndexMode {
        NO_BALL,
        BALL
    }   

    private IndexMode currentMode = IndexMode.NO_BALL;
    private final IndexIO io;
    private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  public Index(IndexIO io) {
    this.io = io;
  }

   private void checkMode() {
    if (inputs.spinnerMotorCurrentAmps >= Constants.IndexConstants.spinnerAmpsLimit) {
      currentMode = IndexMode.BALL;
    } else {
      currentMode = IndexMode.NO_BALL;
    }
  }

  public void setSpinnerVoltage(double volts) {
    io.setSpinnerVoltage(volts);
  }

  private void runStateMachine() {
  switch (currentMode) {

    case NO_BALL:
      setSpinnerVoltage(3.0);
      break;

    case BALL:
      setSpinnerVoltage(10.0);
      break;
  }
}

  public Command setSpinnerVoltageCommand(double volts) {
    return this.runEnd(() -> setSpinnerVoltage(volts), () -> setSpinnerVoltage(0.0));
  }
  

  @Override
public void periodic() {
  
  io.updateInputs(inputs); 
  checkMode();               
  runStateMachine();     

  Logger.processInputs("Index", inputs);
}

  @Override
  public void simulationPeriodic() {
  }
}