package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }

  public Command setRollerVoltageCommand(double volts) {
    return this.runEnd(() -> setRollerVoltage(volts), () -> setRollerVoltage(0.0));
  }

  public Command zeroKickerCommand() {
    return new SequentialCommandGroup(
        setRollerVoltageCommand(Constants.KickerConstants.zeroVoltage),
        new WaitCommand(Constants.KickerConstants.zeroWaitSeconds)).andThen(() -> zeroEncoder());
  }

  public Command zeroEncoderCommand() {
    return this.runOnce(() -> zeroEncoder());
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