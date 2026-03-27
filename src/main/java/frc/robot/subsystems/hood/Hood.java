package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive.RobotZone;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkTunablePID;
import frc.robot.util.SparkTunablePID.DriverType;
import frc.robot.util.rebuilt.field.Field;
import frc.robot.util.rebuilt.field.FieldHelpers;

public class Hood extends SubsystemBase {

    public enum SystemState {
        IDLE,
        POSITION,
        TESTING,
        MANUAL,
        ZEROING
    }

    private SystemState systemState = SystemState.IDLE;

    private double targetPositionDeg = 0.0;
    private LoggedTunableNumber setpoint = new LoggedTunableNumber("hoodsetpoint", 0);

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final SparkTunablePID tSparkTunablePID;

    public Hood(HoodIO io) {
        this.io = io;
        tSparkTunablePID = new SparkTunablePID(io.getMotor(), "Hood", DriverType.MAX, 2, 0, 0);
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

    public void zeroEncoder() {
        systemState = SystemState.IDLE;
        io.setEncoder(0);
    }

    public void setEncoder() {
        io.setEncoder(0);
    }

    public void setVoltage(double vv) {
        io.setAppliedVoltage(vv);
    }

    @Override
    public void periodic() {
        double hoodSetpoint = getHoodSetpoint();
        SmartDashboard.putNumber("distance", getDistanceNotClamped());
        io.updateInputs(inputs);
        switch (systemState) {
            case MANUAL:
                // io.setPosition(setpoint.get());
                break;
            case POSITION:
                io.setPosition(targetPositionDeg);
                break;

            case IDLE:
            default:
                io.stop();
                break;
            case TESTING:
                io.setAppliedVoltage(-6.0);
                break;
            case ZEROING:
                io.setAppliedVoltage(-2);
        }

        Logger.recordOutput("Hood/SystemState", systemState.toString());
        Logger.recordOutput("Hood/TargetDeg", targetPositionDeg);
        Logger.processInputs("Hood", inputs);
        tSparkTunablePID.periodic();
    }

    public boolean isAtSetpoint() {
        return io.isAtSetpoint();
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

    public double getDistance() {
        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Translation2d target = getTarget();

        Translation2d turretRelativeToRobot = TurretConstants.turretOffset;

        double dx = target.getX() - (robotPose.getX() + turretRelativeToRobot.getX());
        double dy = target.getY() - (robotPose.getY() + turretRelativeToRobot.getY());

        double dist = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        double normDist = MathUtil.clamp(dist, 1, 5.5);
        return normDist;
    }

    public double getDistanceNotClamped() {
        Pose2d robotPose = RobotContainer.getDrive().getPose();
        Translation2d target = getTarget();

        Translation2d turretRelativeToRobot = TurretConstants.turretOffset;

        double dx = target.getX() - (robotPose.getX() + turretRelativeToRobot.getX());
        double dy = target.getY() - (robotPose.getY() + turretRelativeToRobot.getY());

        double dist = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        return dist;
    }

    public double getHoodSetpoint() {
        double hoodRot = 500 * getDistance() / 5.5;
        return hoodRot;
    }
}