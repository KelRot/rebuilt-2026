package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SparkTunablePID;
import frc.robot.util.TalonTunablePID;
import frc.robot.util.SparkTunablePID.DriverType;

public class Intake extends SubsystemBase {

  public enum SystemState {
    IDLE,
    INTAKING,
    OUTTAKING,
    OPENING,
    CLOSING,
    POSITION_CONTROL,
    ZEROING,
    MANUAL,
    TESTING
  }

  private SystemState systemState = SystemState.INTAKING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeVisualizer visualizer = new IntakeVisualizer("Measured", Color.kGreen);
  private SparkTunablePID sparkTunablePID;
  private TalonTunablePID talonTunablePID;

  private double zeroStillTime = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    sparkTunablePID = new SparkTunablePID(this.io.getLeadOpenerMotor(), "IntakeOpener", DriverType.MAX, 0, 0, 0);
    talonTunablePID = new TalonTunablePID(this.io.getRollerMotor(), "IntakeRoller", 0, 0, 0, 1 / 509.3 / 60, 0);

  }

  public void requestState(SystemState wantedState) {
    if (DriverStation.isDisabled() && wantedState != SystemState.ZEROING) {
      return;
    }
    systemState = wantedState;
  }

  public void moveToPosition(double position) {
    io.setOpenerSetPoint(position, ControlType.kPosition);
    systemState = SystemState.POSITION_CONTROL;
  }

  public void manualOpener(double volts) {
    io.setOpenerVoltage(volts);
    systemState = SystemState.MANUAL;
  }

  private void handleIntaking(double rollerVoltage) {
    if (!inputs.isIntakeOpen) {
      io.setRollerVoltage(0.0);
      io.setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition - 5, ControlType.kPosition);
    } else {
      io.setOpenerVoltage(0);
      io.setRollerRPM(rollerVoltage);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    visualizer.update(Degrees.of(inputs.IntakePosition).in(Radians));

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IDLE;
    }
    switch (systemState) {

      case INTAKING:
        handleIntaking(Constants.IntakeConstants.INTAKING_RPM);
        break;

      case OUTTAKING:
        handleIntaking(Constants.IntakeConstants.OUTTAKING_RPM);
        break;

      case OPENING:
        io.setRollerVoltage(0.0);
        io.setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition, ControlType.kPosition);
        if (io.isOpenerAtSetpoint()) {
          io.setOpenerSetPoint(0, ControlType.kVoltage);
          systemState = SystemState.IDLE;
        }
        break;

      case CLOSING:
        io.setRollerVoltage(0.0);
        io.setOpenerSetPoint(Constants.IntakeConstants.intakeClosedPosition, ControlType.kPosition);
        if (io.isOpenerAtSetpoint()) {
          systemState = SystemState.IDLE;
        }
        break;

      case POSITION_CONTROL:
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
          io.setOpenerVoltage(0);
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
      case TESTING:
        if (!isOpened()) {
          io.setOpenerVoltage(1.0);
        } else if (isOpened()) {
          io.setRollerVoltage(1.0);
          io.setOpenerVoltage(0.0);
          break;
        }

        sparkTunablePID.periodic();
        talonTunablePID.periodic();
        Logger.recordOutput("Intake/SystemState", systemState.toString());
        Logger.processInputs("Intake", inputs);
    }
  }

  public boolean isOpened() {
    return inputs.isIntakeOpen;
  }
}
