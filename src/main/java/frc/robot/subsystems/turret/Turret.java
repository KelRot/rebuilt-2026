package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive.RobotZone;
import frc.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkTunablePID;
import frc.robot.util.SparkTunablePID.DriverType;
import frc.robot.util.rebuilt.field.Field;
import frc.robot.util.rebuilt.field.FieldHelpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.concurrent.TransferQueue;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretVisualizer visualizer = new TurretVisualizer("Measured", Color.kRed);
    private SparkTunablePID tunablePID;
    private double targetAngle, targetFF;

    private LoggedTunableNumber setpoint;
    private double bestAngle = 0.0;
    private final double minAngle = 0;
    private final double maxAngle = 360;
    private final double CAPACITY = 10;
    private double fuelStored = 0;
    private boolean zeroed = false;

    public Turret(TurretIO io) {
        this.io = io;
        tunablePID = new SparkTunablePID(io.getMotor(), "Turret", DriverType.VORTEX, 0.012, 0, 0.7);
    }

    public static enum SystemState {
        IDLE,
        TRACKING,
        SHOOTING,
        POSITION,
        TESTING
    }


    private SystemState systemState = SystemState.IDLE;

    public double manual_setpoint;
    public double hub_setpoint;

    public void requestState(SystemState wantedState) {
        systemState = wantedState;
    }

    public void setPosition(double position) {
        requestState(SystemState.POSITION);
        manual_setpoint = position;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    @Override
    public void periodic() {
        hub_setpoint = angleToTarget();
        if (zeroed == false) {
            setInitialMotorPosition();
        }
        if (Math.abs(io.getMotor().getEncoder().getPosition() - io.getAngleCalculator().getAngle().getDegrees()) < 3) {
            zeroed = true;
        }
        if (Math.abs(io.getMotor().getEncoder().getPosition() - io.getAngleCalculator().getAngle().getDegrees()) > 3
                && io.getAngleCalculator().isHealthy()) {
            io.setEncoder(io.getAngleCalculator().getAngle().getDegrees());
        }
        io.updateInputs(inputs);

        switch (systemState) {

            case IDLE:
                io.setVoltage(0);
                break;

            case TRACKING:
                io.setPosition(hub_setpoint);
                break;

            case SHOOTING:
                io.setPosition(targetAngle);
                break;

            case POSITION:
                io.setPosition(setpoint.get());
                break;

            case TESTING:
                io.setVoltage(0.2);
                break;
        }
        SmartDashboard.putNumber("turretdegree", io.getAngleCalculator().getAngle().getDegrees());
        visualizer.update(inputs.positionRads, hub_setpoint);
        tunablePID.periodic();
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/SystemState", systemState.toString());
        Logger.recordOutput("Turret/Target", getTarget());
    }

    @Override
    public void simulationPeriodic() {
        visualizer.updateFuel(
                LinearVelocity.ofBaseUnits(8.0, MetersPerSecond),
                Degrees.of(30));
    }

    public boolean isAtSetpoint() {
        return io.isAtSetpoint();
    }

    public void setInitialMotorPosition() {
        io.setEncoder(io.getAngleCalculator().getAngle().getDegrees());
        io.setPosition(io.getAngleCalculator().getAngle().getDegrees());
        setpoint = new LoggedTunableNumber("turretsetpoint", io.getAngleCalculator().getAngle().getDegrees());
    }

    public void stop() {
        io.stop();
    }

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel() {
        if (fuelStored == 0)
            return;

        fuelStored--;

        TurretIOSim.getFuelSim().launchFuel(
                LinearVelocity.ofBaseUnits(8, MetersPerSecond),
                Degrees.of(30),
                Degrees.of(RobotContainer.getDrive().getPose().getRotation().getDegrees()
                        + RobotContainer.getTurret().angleToTarget()),
                Distance.ofBaseUnits(0.262, Meters));
    }

    public Translation2d getTarget() {

        RobotZone robotPose = RobotContainer.getDrive().getRobotZone();

        switch (robotPose) {

            case UPPER_NEUTRAL_ZONE -> {
                return new Translation2d(FieldHelpers.flipXifRed(2.510), 6);
            }

            case LOWER_NEUTRAL_ZONE -> {
                return new Translation2d(FieldHelpers.flipXifRed(2.510), 2);
            }

            case BLUE_ALLIANCE_ZONE -> {

                if (Field.isRed()) {

                    if (RobotContainer.getDrive().getPose().getY() < Field.getBlueHubCenter().getY()) {
                        return new Translation2d(FieldHelpers.flipXifRed(2.510), 2);
                    } else {
                        return new Translation2d(FieldHelpers.flipXifRed(2.510), 6);
                    }

                }

                return Field.getBlueHubCenter().toTranslation2d();
            }

            case RED_ALLIANCE_ZONE -> {

                if (Field.isBlue()) {

                    if (RobotContainer.getDrive().getPose().getY() < Field.getRedHubCenter().getY()) {
                        return new Translation2d(FieldHelpers.flipXifRed(2.510), 2);
                    } else {
                        return new Translation2d(FieldHelpers.flipXifRed(2.510), 6);
                    }

                }

                return Field.getRedHubCenter().toTranslation2d();
            }

            default -> {
                return new Translation2d(0.0, 0.0);
            }
        }
    }

    public double angleToTarget() {

        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Translation2d target = getTarget();

        Translation2d turretRelativeToRobot = TurretConstants.turretOffset;

        double dx = target.getX() - (robotPose.getX() + turretRelativeToRobot.getX());
        double dy = target.getY() - (robotPose.getY() + turretRelativeToRobot.getY());

        double targetRad = Math.atan2(dy, dx);

        double turretRad = Degrees.convertFrom(targetRad, Radians) - robotPose.getRotation().getDegrees() - 90;

        if (turretRad < minAngle) {
            turretRad += 360;
        }
        if (turretRad > maxAngle) {

            turretRad -= 360;
        }
        bestAngle = turretRad;

        return bestAngle;
    }
}