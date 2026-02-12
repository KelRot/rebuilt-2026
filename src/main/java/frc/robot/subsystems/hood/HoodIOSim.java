package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodIOSim implements HoodIO {
    private final DCMotorSim hoodMotorSim;

    private PIDController hoodController = new PIDController(kP, 0, kD);
    private DutyCycleEncoderSim absEncoder1Sim = new DutyCycleEncoderSim(HoodConstants.absEncoder1ID);
    private DutyCycleEncoderSim absEncoder2Sim = new DutyCycleEncoderSim(HoodConstants.absEncoder2ID);
    private boolean hoodClosedLoop = false;
    private double hoodAppliedVolts = 0.0;

    public HoodIOSim(){
        hoodMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(HoodConstants.hoodGearbox, 0.04, HoodConstants.hoodMotorReduction), HoodConstants.hoodGearbox);
        hoodController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs){
        if(hoodClosedLoop){
            hoodAppliedVolts = hoodController.calculate(hoodMotorSim.getAngularPositionRad());
        } else {
            hoodController.reset();
        }

        hoodMotorSim.setInputVoltage(MathUtil.clamp(hoodAppliedVolts, -12.0, 12));
        hoodMotorSim.update(0.02);

        inputs.motorConnected = true;
        inputs.positionRads = hoodMotorSim.getAngularPositionRad();
        inputs.absPositionTours1 = absEncoder1Sim.get();
        inputs.absPositionTours2 = absEncoder2Sim.get();
        inputs.velocityRadsPerSec = hoodMotorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = hoodAppliedVolts;
        inputs.supplyCurrentAmps = Math.abs(hoodMotorSim.getCurrentDrawAmps());

    }

    @Override
    public void setVoltage(double voltage){
        hoodClosedLoop = false;
        hoodAppliedVolts = voltage; 
    }

    @Override
    public void setPosition(double setpoint){
        hoodClosedLoop = true;
        hoodController.setSetpoint(setpoint);
    }

}