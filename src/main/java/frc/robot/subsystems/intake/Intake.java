package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

  /** Creates a new ExampleSubsystem. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  public void setOpenerSetPoint(double setPoint) {
    io.setOpenerSetPoint(setPoint);
  }

  public void setOpenerVoltage(double volts) {
    io.setOpenerVoltage(volts);
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }

  public Command setRollerVoltageCommand(double volts) {
    return this.runEnd(() -> setRollerVoltage(volts), () -> setRollerVoltage(0.0));
  }

  public Command setOpenerSetPointCommand(double setPoint) {
    return this.runOnce(() -> setOpenerSetPoint(setPoint))
        .until(() -> Math.abs(inputs.IntakePosition - setPoint) < 2.0).andThen(() -> setOpenerVoltage(0));
  }

  public Command setOpenerVoltageCommand(double volts) {
    return this.runEnd(() -> setOpenerVoltage(volts), () -> setOpenerVoltage(0.0));
  }

  public Command zeroIntakeCommand() {
    return new SequentialCommandGroup(
        setOpenerVoltageCommand(Constants.IntakeConstants.zeroVoltage),
        new WaitCommand(Constants.IntakeConstants.zeroWaitSeconds)).andThen(() -> zeroEncoder());
  }

  public Command zeroEncoderCommand() {
    return this.runOnce(() -> zeroEncoder());
  }

  @Override
  public void periodic() {
    measuredVisualizer.setAngleDeg(inputs.IntakePosition);
    measuredVisualizer.update();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void simulationPeriodic() {
  }
}
