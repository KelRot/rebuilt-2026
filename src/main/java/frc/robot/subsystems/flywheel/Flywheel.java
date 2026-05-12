package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkTunablePID;
import frc.robot.util.SparkTunablePID.DriverType;

public class Flywheel extends SubsystemBase {

    /* ---------------- State ---------------- */

    public enum SystemState {
        IDLE,
        PASSIVE,
        TARGET_RPM,
        VOLTAGE,
        TESTING

    }

    private SystemState systemState = SystemState.IDLE;

    private double targetRpm = 0.0;

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final SparkTunablePID sparkTunablePID;
    private final LoggedTunableNumber setpoint = new LoggedTunableNumber("flywheelsetpoint", 0);
    private final LoggedTunableNumber voltsetpoint = new LoggedTunableNumber("flywheelvoltsetpoint", 0);
    public Flywheel(FlywheelIO io) {
        this.io = io;
        sparkTunablePID = new SparkTunablePID(io.getMotor(), "Flywheel", DriverType.VORTEX, 0.00019, 0, 0.008);
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

    public void setVoltage(){
        io.setAppliedVoltage(voltsetpoint.get());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        sparkTunablePID.periodic();
        
        switch (systemState) {

            case TARGET_RPM:
                io.setRpm(targetRpm);
                break;

            case PASSIVE:
                io.setRpm(500.0); // standby spin
                break;
            case VOLTAGE: 
                setVoltage();
                break;
            case IDLE:
            default:
                io.setAppliedVoltage(0.0);
                break;
            case TESTING:
                io.setRpm(setpoint.get());;
                break;
        }
        inputs.isAtSetpoint = isAtSetpoint();
        Logger.recordOutput("Flywheel/SystemState", systemState.toString());
        Logger.processInputs("Flywheel", inputs);
    }
        public boolean isAtSetpoint() {
            if (systemState == SystemState.IDLE){
                return false;
            }
            else{
                double currentRPM = io.getLeadVelocityRpm();
                return Math.abs(currentRPM - targetRpm) < 500;
        }
}
}