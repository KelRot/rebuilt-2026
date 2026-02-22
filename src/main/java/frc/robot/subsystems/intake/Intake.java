package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public enum SystemState {
    IDLE,
    INTAKING,
    OUTTAKING,
    OPENING,
    CLOSING,
    POSITION_CONTROL,
    ZEROING,
    MANUAL
  }

  

  private SystemState systemState = SystemState.INTAKING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeVisualizer visualizer = new IntakeVisualizer("Measured", Color.kGreen);

  private double zeroStillTime = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void requestState(SystemState wantedState) {
    if (DriverStation.isDisabled() && wantedState != SystemState.ZEROING) {
      return;
    }
    systemState = wantedState;
  }

  public void moveToPosition(double position) {
    io.setOpenerSetPoint(position);
    systemState = SystemState.POSITION_CONTROL;
  }

  public void manualOpener(double volts) {
    io.setOpenerVoltage(volts);
    systemState = SystemState.MANUAL;
  }

  private void handleIntaking(double rollerVoltage) {
    if (!inputs.isIntakeOpen) {
      io.setRollerVoltage(0.0);
      io.setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition);
    } else {
      io.setRollerVoltage(rollerVoltage);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    visualizer.update(inputs.IntakePosition);

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IDLE;
    }

    switch (systemState) {

      case INTAKING:
        handleIntaking(Constants.IntakeConstants.INTAKING_VOLTAGE);
        break;

      case OUTTAKING:
        handleIntaking(Constants.IntakeConstants.OUTTAKING_VOLTAGE);
        break;

      case OPENING:
        io.setRollerVoltage(0.0);
        io.setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition);
        if (io.isOpenerAtSetpoint()) {
          systemState = SystemState.IDLE;
        }
        break;

      case CLOSING:
        io.setRollerVoltage(0.0);
        io.setOpenerSetPoint(Constants.IntakeConstants.intakeClosedPosition);
        if (io.isOpenerAtSetpoint()) {
          systemState = SystemState.IDLE;
        }
        break;

      case POSITION_CONTROL:
        io.setRollerVoltage(0.0);
        if (io.isOpenerAtSetpoint()) {
          io.setOpenerVoltage(0.0);
          systemState = SystemState.IDLE;
        }
        break;

      case ZEROING:
        io.setRollerVoltage(0.0);
        io.setOpenerVoltage(Constants.IntakeConstants.zeroVoltage);

        if (Math.abs(inputs.openerMotorVelocityRPS) < Constants.IntakeConstants.ZERO_VELOCITY_EPS) {
          zeroStillTime += 0.02;
        } else {
          zeroStillTime = 0.0;
        }

        if (zeroStillTime >= Constants.IntakeConstants.ZERO_CONFIRM_TIME) {
          io.setOpenerVoltage(0.0);
          io.zeroEncoder();
          zeroStillTime = 0.0;
          systemState = SystemState.IDLE;
        }
        break;

      case MANUAL:
        io.setRollerVoltage(0.0);
        break;

      case IDLE:
      default:
        io.setRollerVoltage(0.0);
        io.setOpenerVoltage(0.0);
        break;
    }

    Logger.recordOutput("Intake/SystemState", systemState.toString());
    Logger.processInputs("Intake", inputs);
  }
}
