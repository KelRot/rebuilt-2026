package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    /* ---------------- State ---------------- */

    public enum SystemState {
        IDLE,
        PASSIVE,
        TARGET_RPM
    }

    private SystemState systemState = SystemState.IDLE;

    private double targetRpm = 0.0;

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void requestState(SystemState wantedState) {
        systemState = wantedState;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        systemState = SystemState.TARGET_RPM;
    }

    public void stop() {
        systemState = SystemState.IDLE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (systemState) {

            case TARGET_RPM:
                io.setRpm(targetRpm);
                break;

            case PASSIVE:
                io.setRpm(500.0); // standby spin
                break;

            case IDLE:
            default:
                io.setRpm(0.0);
                break;
        }
        inputs.isAtSetpoint = io.isAtSetpoint();
        Logger.recordOutput("Flywheel/SystemState", systemState.toString());
        Logger.processInputs("Flywheel", inputs);
    }
}
