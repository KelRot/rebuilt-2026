package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.kicker.KickerIO.KickerIOInputs;

public class Kicker extends SubsystemBase {

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  public Command setRollerVoltageCommand(double volts) {
    return this.runEnd(() -> setRollerVoltage(volts), () -> setRollerVoltage(0.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  @Override
  public void simulationPeriodic() {
  }
}
