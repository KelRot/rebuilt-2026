package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    public enum SystemState {
        IDLE,
        POSITION,
        TESTING
    }

    private SystemState systemState = SystemState.IDLE;

    private double targetPositionDeg = 0.0;

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    public void requestState(SystemState wantedState) {
        systemState = wantedState;
    }

    public void setTargetPositionDeg(double positionDeg) {
        targetPositionDeg = positionDeg;
        systemState = SystemState.POSITION;
    }

    public void stop() {
        systemState = SystemState.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (systemState) {

            case POSITION:
                io.setPosition(targetPositionDeg);
                break;

            case IDLE:
            default:
                io.stop();
                break;
            case TESTING:
                io.setAppliedVoltage(1.0);
                break;
        }

        inputs.isAtSetpoint = io.isAtSetpoint();
        Logger.recordOutput("Hood/SystemState", systemState.toString());
        Logger.recordOutput("Hood/TargetDeg", targetPositionDeg);
        Logger.processInputs("Hood", inputs);
    }
      public boolean isAtSetpoint() {
        return io.isAtSetpoint();
      }
    }