
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {

    /* ---------------- Constants ---------------- */

    private static final double LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;


    /* ---------------- Simulation ---------------- */

    private final DCMotor openerMotor = DCMotor.getNEO(1);
    private static final double MIN_ANGLE_RAD = Degrees.of(0).in(Radians);
    private static final double MAX_ANGLE_RAD = Degrees.of(110).in(Radians);
    private static final double ARM_LENGTH_METERS = 0.48; // ~19 in
    private static final double GEAR_RATIO = 10.0;

    // Sim
    private final SingleJointedArmSim openerSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                    openerMotor,
                    0.12, // moment of inertia (kg·m²)
                    ARM_LENGTH_METERS),
            openerMotor,
            GEAR_RATIO,
            ARM_LENGTH_METERS,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            true, // gravity ON (intake düşer)
            MIN_ANGLE_RAD // start closed
    );

    private final DCMotor rollerMotor = DCMotor.getNEO(1);
    private final DCMotorSim rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rollerMotor, 0.0005, 3),
            rollerMotor);


    @Override
    public void setOpenerSetPoint(double anglesDegrees) {
        double anglesRadians = Degrees.of(anglesDegrees).in(Radians);
        double openerSetpointRads = MathUtil.clamp(anglesRadians, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        openerSim.setState(openerSetpointRads, 0.0);
    }

    @Override
    public void setOpenerVoltage(double volts) {
        openerSim.setInputVoltage(
                MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE));
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerSim.setInputVoltage(
                MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE));
    }

    @Override
    public void stopAllMotors() {
        openerSim.setInputVoltage(0.0);
        rollerSim.setInputVoltage(0.0);
    }

    @Override
    public void zeroEncoder() {
        openerSim.setState(0.0, 0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        /* ---- Step simulation ---- */
        openerSim.update(LOOP_PERIOD);
        rollerSim.update(LOOP_PERIOD);

        /* ---- Inputs ---- */
        inputs.openerConnected = true;
        inputs.rollerConnected = true;

        inputs.IntakePosition = Radians.of(openerSim.getAngleRads()).in(Degrees);
        inputs.openerMotorCurrentAmps = openerSim.getCurrentDrawAmps();

        inputs.rollerMotorSpeedRpm = rollerSim.getAngularVelocityRPM();
        inputs.rollerMotorCurrentAmps = rollerSim.getCurrentDrawAmps();
        inputs.rollerMotorVoltageVolts = rollerSim.getInputVoltage();

    }

}
