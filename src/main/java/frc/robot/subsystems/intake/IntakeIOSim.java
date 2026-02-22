package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {

    private static final double LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;

    private static final double MIN_ANGLE_RAD = Degrees.of(0).in(Radians);
    private static final double MAX_ANGLE_RAD = Degrees.of(180).in(Radians);

    private static final double ARM_LENGTH_METERS = 0.48;
    private static final double GEAR_RATIO = 10.0;

    private static final double ZERO_VELOCITY_RAD_PER_SEC = 0.05;
    private static final double ZERO_ANGLE_EPSILON_RAD = Degrees.of(2).in(Radians);
    private static final double ZERO_TIME_REQUIRED = 0.25; // seconds

    private final DCMotor openerMotor = DCMotor.getNEO(1);
    private final SingleJointedArmSim openerSim = new SingleJointedArmSim(
            openerMotor,
            GEAR_RATIO,
            0.12,
            ARM_LENGTH_METERS,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            true,
            MIN_ANGLE_RAD);

    private final DCMotor rollerMotor = DCMotor.getNEO(1);
    private final DCMotorSim rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerMotor, 0.0005, 1),
            rollerMotor);

    private double zeroTimerSeconds = 0.0;
    private boolean isZeroed = true;

    @Override
    public void setOpenerSetPoint(double degrees) {
        double radians = MathUtil.clamp(Radians.of(degrees).in(Radians), MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        openerSim.setState(radians, 0.0);
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
        isZeroed = true;
        zeroTimerSeconds = 0.0;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        openerSim.update(LOOP_PERIOD);
        rollerSim.update(LOOP_PERIOD);

        double angle = openerSim.getAngleRads();
        double velocity = Math.abs(openerSim.getVelocityRadPerSec());

        boolean nearZeroAngle = Math.abs(angle - MIN_ANGLE_RAD) < ZERO_ANGLE_EPSILON_RAD;

        boolean stopped = velocity < ZERO_VELOCITY_RAD_PER_SEC;

        if (nearZeroAngle && stopped) {
            zeroTimerSeconds += LOOP_PERIOD;
            if (zeroTimerSeconds >= ZERO_TIME_REQUIRED) {
                isZeroed = true;
            }
        } else {
            zeroTimerSeconds = 0.0;
            isZeroed = false;
        }

        inputs.openerConnected = true;
        inputs.rollerConnected = true;

        inputs.IntakePosition = Radians.of(angle).in(Degrees);

        inputs.isIntakeOpen = angle > Degrees.of(135).in(Radians);

        inputs.isZeroed = isZeroed;

        inputs.openerMotorCurrentAmps = openerSim.getCurrentDrawAmps();

        inputs.rollerMotorSpeedRpm = rollerSim.getAngularVelocityRPM();
        inputs.rollerMotorCurrentAmps = rollerSim.getCurrentDrawAmps();
        inputs.rollerMotorVoltageVolts = rollerSim.getInputVoltage();
    }
}
