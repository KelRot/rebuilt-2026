package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
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

  private SystemState systemState = SystemState.IDLE;

  private final Drive drive;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeVisualizer visualizer = new IntakeVisualizer("Measured", Color.kGreen);
  private final LoggedTunableNumber setpoint = new LoggedTunableNumber("rollersetpoint", 0);
  private SparkTunablePID sparkTunablePID;
  private TalonTunablePID talonTunablePID;

  private double zeroStillTime = 0.0;
  private boolean isZeroed = false;

  public Intake(IntakeIO io, Drive drive) {
   
    this.io = io;
    sparkTunablePID = new SparkTunablePID(this.io.getLeadOpenerMotor(), "IntakeOpener", DriverType.MAX, 0.011, 0, 0.65);
    talonTunablePID = new TalonTunablePID(this.io.getRollerMotor(), "IntakeRoller", 0.002, 0, 0, 0.141, 0);
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    this.drive = drive;
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

  private void handleIntaking(double rollerRpm) {
    if (!inputs.isIntakeOpen) {
      io.setRollerVoltage(0.0);
      io.setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition);
    } else {
      io.setOpenerVoltage(0);
      io.setRollerRPM(rollerRpm);
    }
  }

  @Override
  public void periodic() {
    if(isOpened()) {
      io.setBrake(false);
    } else {
      io.setBrake(true);
    }
    io.updateInputs(inputs);
    visualizer.update(Degrees.of(inputs.IntakePosition).in(Radians));

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IDLE;
    }
    switch (systemState) {

      case INTAKING:
      handleIntaking(
        drive.getChassisSpeeds().vxMetersPerSecond < 0
        ? 2000 * drive.getChassisSpeeds().vxMetersPerSecond / DriveConstants.maxSpeedMetersPerSec + Constants.IntakeConstants.INTAKING_RPM
        : Constants.IntakeConstants.INTAKING_RPM);

      case OUTTAKING:
        handleIntaking(Constants.IntakeConstants.OUTTAKING_RPM);
        break;

      case OPENING:
        io.setRollerVoltage(0.0);
        io.setOpenerSetPoint(Constants.IntakeConstants.intakeOpenPosition);
        if (io.isOpenerAtSetpoint()) {
          io.setOpenerVoltage(0);
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
        io.setRollerRPM(setpoint.get());
        //io.setOpenerSetPoint(setpoint.get());
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
    }

    sparkTunablePID.periodic();
    talonTunablePID.periodic();
    

    Logger.recordOutput("Intake/SystemState", systemState.toString());
    Logger.processInputs("Intake", inputs);
  }

  public void setZeroed(boolean zeroed) {
    isZeroed = zeroed;
  }
  public void setEncoder() {
    io.getLeadOpenerMotor().getEncoder().setPosition(0);
  }
  public boolean isOpened() {
    return inputs.isIntakeOpen;
  }
}
