package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

  public IntakeSystemState systemState = IntakeSystemState.OFF;

  public enum IntakeSystemState {
    INTAKING,
    OUTTAKING,
    POSITION_CONTROL,
    ZEROING,
    MANUAL_CONTROL,
    OFF
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /* ---------------- Low-level setters ---------------- */

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

  /* ---------------- Commands ---------------- */

  public Command setRollerVoltageCommand(double volts) {
    return this.startEnd(
        () -> setRollerVoltage(volts),
        () -> setRollerVoltage(0.0));
  }

  public Command intakeCommand() {
    return this.startEnd(
        () -> systemState = IntakeSystemState.INTAKING,
        () -> systemState = IntakeSystemState.OFF);
  }

  public Command outtakeCommand() {
    return this.startEnd(
        () -> systemState = IntakeSystemState.OUTTAKING,
        () -> systemState = IntakeSystemState.OFF);
  }

  public Command setOpenerSetPointCommand(double setPoint) {
    return this.runOnce(() -> {
      systemState = IntakeSystemState.POSITION_CONTROL;
      setOpenerSetPoint(setPoint);
    })
        .until(io::isOpenerAtSetpoint)
        .finallyDo(() -> {
          setOpenerVoltage(0.0);
          systemState = IntakeSystemState.OFF;
        });
  }

  public Command setOpenerVoltageCommand(double volts) {
    return this.startEnd(
        () -> {
          systemState = IntakeSystemState.MANUAL_CONTROL;
          setOpenerVoltage(volts);
        },
        () -> {
          setOpenerVoltage(0.0);
          systemState = IntakeSystemState.OFF;
        });
  }

  public Command zeroIntakeCommand() {
    return this.run(() -> {
      systemState = IntakeSystemState.ZEROING;
      setOpenerVoltage(Constants.IntakeConstants.zeroVoltage);
    })
        .withTimeout(Constants.IntakeConstants.zeroWaitSeconds)
        .finallyDo(() -> {
          setOpenerVoltage(0.0);
          zeroEncoder();
          systemState = IntakeSystemState.OFF;
          setRollerVoltage(0.0);
        });
  }

  public Command openIntakeCommandWithPosition() {
    return this.runOnce(() -> setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition))
        .until(io::isOpenerAtSetpoint)
        .finallyDo(() -> setOpenerVoltage(0.0));
  }

  public Command openIntakeCommandWithVoltage() {
    return this.run(() -> {
      setOpenerVoltage(Constants.IntakeConstants.openVoltage);
    })
        .withTimeout(Constants.IntakeConstants.openWaitSeconds)
        .finallyDo(() -> {
          setOpenerVoltage(0.0);
          zeroEncoder();
        });
  }

  /* ---------------- Periodic ---------------- */

  @Override
  public void periodic() {
    // ALWAYS FIRST
    io.updateInputs(inputs);

    measuredVisualizer.setAngleDeg(inputs.IntakePosition);
    measuredVisualizer.update();

    switch (systemState) {
      case INTAKING:
        if (inputs.isIntakeOpen) {
          setRollerVoltage(Constants.IntakeConstants.INTAKING_VOLTAGE);
        } else {
          openIntakeCommandWithPosition().schedule();
        }
        break;

      case OUTTAKING:
        if (inputs.isIntakeOpen) {
          setRollerVoltage(Constants.IntakeConstants.OUTTAKING_VOLTAGE);
        } else {
          openIntakeCommandWithPosition().schedule();
        }
        break;

      case POSITION_CONTROL:
        // Motor PID assumed to be inside IO
        break;

      case ZEROING:
      case MANUAL_CONTROL:
        // Voltage is driven by active command
        break;

      case OFF:
      default:
        setRollerVoltage(0.0);
        break;
    }

    Logger.recordOutput("Intake/SystemState", systemState.toString());
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void simulationPeriodic() {
  }
}
