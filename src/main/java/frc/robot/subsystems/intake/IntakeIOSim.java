package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private static final double LOOP_PERIOD = 0.02;
  private static final double MAX_VOLTAGE = 12.0;


  private static final double MECH_ZERO_RAD =
      Degrees.of(-37).in(Radians);

  private static final double MIN_MECH_ANGLE_RAD =
      Degrees.of(-150).in(Radians) + MECH_ZERO_RAD;

  private static final double MAX_MECH_ANGLE_RAD =
      MECH_ZERO_RAD;

  private static final double ARM_LENGTH_METERS = 0.48;
  private static final double GEAR_RATIO = 10.0;
  private static final double ARM_MOI = 0.12;


  private final SingleJointedArmSim openerSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          GEAR_RATIO,
          ARM_MOI,
          ARM_LENGTH_METERS,
          MIN_MECH_ANGLE_RAD,
          MAX_MECH_ANGLE_RAD,
          true,             
          MECH_ZERO_RAD);

  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), 0.0005, 1),
          DCMotor.getNEO(1));


  private final PIDController pid =
      new PIDController(8, 2, 1);

  private double targetMechRad = MECH_ZERO_RAD;
  private boolean isZeroed = true;

  public IntakeIOSim() {

    openerSim.setState(MECH_ZERO_RAD, 0.0);
    pid.setTolerance(Degrees.of(2).in(Radians));
  }

  @Override
  public void setOpenerSetPoint(double encoderDegrees) {

    targetMechRad =
        Degrees.of(encoderDegrees).in(Radians)
            + MECH_ZERO_RAD;

    targetMechRad =
        MathUtil.clamp(
            targetMechRad,
            MIN_MECH_ANGLE_RAD,
            MAX_MECH_ANGLE_RAD);
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
    openerSim.setState(MECH_ZERO_RAD, 0.0);
    isZeroed = true;
  }


  @Override
  public void updateInputs(IntakeIOInputs inputs) {


    double pidOut =
        pid.calculate(
            openerSim.getAngleRads(),
            targetMechRad);

    double volts =
        MathUtil.clamp(pidOut, -MAX_VOLTAGE, MAX_VOLTAGE);

    openerSim.setInputVoltage(volts);

    openerSim.update(LOOP_PERIOD);
    rollerSim.update(LOOP_PERIOD);


    inputs.openerConnected = true;
    inputs.rollerConnected = true;

    inputs.IntakePosition =
        Radians.of(
            openerSim.getAngleRads() - MECH_ZERO_RAD)
            .in(Degrees);

    inputs.openerMotorCurrentAmps =
        openerSim.getCurrentDrawAmps();

    inputs.rollerMotorSpeedRpm =
        rollerSim.getAngularVelocityRPM();

    inputs.rollerMotorCurrentAmps =
        rollerSim.getCurrentDrawAmps();

    inputs.rollerMotorVoltageVolts =
        rollerSim.getInputVoltage();
    inputs.isIntakeOpen = inputs.IntakePosition < Constants.IntakeConstants.intakeOpenPosition + 10;
    inputs.isZeroed = isZeroed;
  }
}