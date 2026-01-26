package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

    public enum IndexMode {
        NO_BALL,
        BALL
    }   

    private IndexMode currentMode = IndexMode.NO_BALL;
    private final IndexIO io;
    private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();
    private final IndexVisualizer measuredVisualizer = new IndexVisualizer("Measured", Color.kRed); // Picked Red for index visualizer (may change later)

  public Index(IndexIO io) {
    this.io = io;
  }

   private void checkMode() {
    if (inputs.spinnerMotorCurrentAmps >= 15.0) {
      currentMode = IndexMode.BALL;
    } else {
      currentMode = IndexMode.NO_BALL;
    }
  }

  public void setSpinnerVoltage(double volts) {
    io.setSpinnerVoltage(volts);
  }

  public void zeroEncoder() {
    io.zeroEncoder();
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

  public Command zeroEncoderCommand() {
    return this.runOnce(() -> zeroEncoder());
  }
  

  @Override
public void periodic() {
  
  io.updateInputs(inputs); 
  checkMode();               
  runStateMachine();     

  measuredVisualizer.update();
  Logger.processInputs("Index", inputs);
}

  @Override
  public void simulationPeriodic() {
  }
}