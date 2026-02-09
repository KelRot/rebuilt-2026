package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim turretMotorSim;

    private PIDController turretController = new PIDController(kP, 0, kD);
    private DutyCycleEncoderSim absEncoder1Sim = new DutyCycleEncoderSim(TurretConstants.absEncoder1ID);
    private DutyCycleEncoderSim absEncoder2Sim = new DutyCycleEncoderSim(TurretConstants.absEncoder2ID);
    private boolean turretClosedLoop = false;
    private double turretFFVolts = 0.0;
    private double turretAppliedVolts = 0.0;

    public TurretIOSim(){
        turretMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(TurretConstants.turretGearbox, 0.04, TurretConstants.turretMotorReduction), TurretConstants.turretGearbox);
        turretController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        if(turretClosedLoop){
            turretAppliedVolts = turretFFVolts + turretController.calculate(turretMotorSim.getAngularPositionRad());
        } else {
            turretController.reset();
        }

        turretMotorSim.setInputVoltage(MathUtil.clamp(turretAppliedVolts, -12.0, 12));
        turretMotorSim.update(0.02);

        inputs.motorConnected = true;
        inputs.positionRads = turretMotorSim.getAngularPositionRad();
        inputs.absPositionTours1 = absEncoder1Sim.get();
        inputs.absPositionTours2 = absEncoder2Sim.get();
        inputs.velocityRadsPerSec = turretMotorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = turretAppliedVolts;
        inputs.supplyCurrentAmps = Math.abs(turretMotorSim.getCurrentDrawAmps());

    }

    @Override
    public void setVoltage(double voltage){
        turretClosedLoop = false;
        turretAppliedVolts = voltage;
    }

    @Override
    public void setPosition(double setpoint){
        turretClosedLoop = true;
        turretController.setSetpoint(setpoint);
    }

}