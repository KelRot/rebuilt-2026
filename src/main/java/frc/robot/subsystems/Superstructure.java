package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.RobotZone;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.rebuilt.field.Field;
import frc.robot.util.rebuilt.field.FieldHelpers;
import frc.robot.util.shotutil.ShotCalculator;
import frc.robot.util.shotutil.ShotLUT;
import frc.robot.util.shotutil.ShotParameters;
import lombok.Getter;

public class Superstructure extends SubsystemBase {

    /* ================= DEPENDENCIES ================= */

    private final Intake intake;
    private final Flywheel flywheel;
    private final Kicker kicker;
    private final Hood hood;
    private final Turret turret;
    private final Drive drive;
    private final Index index;

    @Getter
    private final ShotLUT lut = new ShotLUT();

    private ShotCalculator.Config config = new ShotCalculator.Config();

    private ShotCalculator shotCalc;

    private Drive swerve = RobotContainer.getDrive();

    public ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            swerve.getPose(),
            swerve.getChassisSpeeds(),
            swerve.getFieldSpeeds(),
            0.9// vision confidence, 0 to 1
    );

    /* ================= STATE ================= */

    private SuperstructureState currentState = SuperstructureState.IDLE;
    private SuperstructureState wantedState = SuperstructureState.IDLE;

    /* ================= ENUM ================= */

    public enum SuperstructureState {
        OPENING_INTAKE,
        INTAKING,
        CLOSING_AND_STOPPING_INTAKE,
        REJECTING_INTAKE,

        STUCKED_RECOVERY,

        PREP_SHOOTING,
        PREP_SHOOTING_AND_INTAKING,
        SHOOTING,
        SHOOTING_AND_INTAKING,

        DEFAULT,
        IDLE,
        TESTING
    }

    /* ================= CONSTRUCTOR ================= */

    public Superstructure(
            Intake intake,
            Flywheel flywheel,
            Kicker kicker,
            Hood hood,
            Turret turret,
            Drive drive,
            Index index) {

        this.intake = intake;
        this.flywheel = flywheel;
        this.kicker = kicker;
        this.hood = hood;
        this.turret = turret;
        this.drive = drive;
        this.index = index;

        config.launcherOffsetX = 0.135; // how far forward the launcher is from robot center (m)
        config.launcherOffsetY = 0.0; // how far left, 0 if centered
        config.phaseDelayMs = 15.0; // your vision pipeline latency
        config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
        config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
        config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
        config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

        shotCalc = new ShotCalculator(config);
        lut.put(5.58, new ShotParameters(-3580, 585, 0.5));
        lut.put(4.9, new ShotParameters(-3400, 400, 0.5));
        lut.put(4.32, new ShotParameters(-3500, 460, 0.5));
        lut.put(2.68, new ShotParameters(-3300, 0, 0.5));
        lut.put(2, new ShotParameters(-2830, 0, 0.5));
        lut.put(3.41, new ShotParameters(-3150, 230, 0.5));
        lut.put(4.05, new ShotParameters(-3300, 300, 0.5));
        lut.put(8.45, new ShotParameters(-3800, 1100, 0.5));
        lut.put(3.1, new ShotParameters(-3200, 100, 0.5));
        lut.put(2.3, new ShotParameters(-3100, 100, 0.5));
        shotCalc.loadShotLUT(lut);

    }

    public void setState(SuperstructureState state) {
        wantedState = state;
    }
    /* ================= CORE LOOP ================= */

    @Override
    public void periodic() {
        if (currentState != wantedState) {
            applyState(wantedState);
            currentState = wantedState;
        }
        if (hood.isAtSetpoint()
                && currentState == SuperstructureState.PREP_SHOOTING) {
            wantedState = SuperstructureState.SHOOTING;
        }
        if (hood.isAtSetpoint()
                && currentState == SuperstructureState.PREP_SHOOTING_AND_INTAKING) {
            wantedState = SuperstructureState.SHOOTING_AND_INTAKING;
        }

        if (isAnyShootingState(currentState)) {
            inputs = new ShotCalculator.ShotInputs(
                    swerve.getPose(),
                    swerve.getChassisSpeeds(),
                    swerve.getFieldSpeeds(),
                    1// vision confidence, 0 to 1
            );
            ShotCalculator.LaunchParameters result = shotCalc.calculate(inputs, getTarget());
            flywheel.setTargetRpm(result.rpm());
            turret.setTargetAngle(result.predictiveTurretAngleDeg());
            hood.setTargetPositionDeg(shotCalc.getHoodAngle(result.solvedDistanceM()));
        }
        Logger.recordOutput("SuperStructure/State", currentState.toString());
    }

    /* ================= STATE APPLY ================= */

    private void applyState(SuperstructureState state) {

        switch (state) {

            case OPENING_INTAKE:
                intake.requestState(Intake.SystemState.OPENING);
                break;

            case INTAKING:
                intake.requestState(Intake.SystemState.INTAKING);
                index.requestState(Index.SystemState.PASSIVE);
                break;

            case CLOSING_AND_STOPPING_INTAKE:
                intake.requestState(Intake.SystemState.CLOSING);
                index.requestState(Index.SystemState.IDLE);
                break;

            case REJECTING_INTAKE:
                intake.requestState(Intake.SystemState.OUTTAKING);
                index.requestState(Index.SystemState.OUTTAKING);
                break;

            case PREP_SHOOTING:
                hood.requestState(Hood.SystemState.POSITION);
                turret.requestState(Turret.SystemState.SHOOTING);
                flywheel.requestState(Flywheel.SystemState.TARGET_RPM);
                
                kicker.requestState(Kicker.SystemState.ENABLED);
                index.requestState(Index.SystemState.INDEXING);
                break;

            case PREP_SHOOTING_AND_INTAKING:
                applyState(SuperstructureState.PREP_SHOOTING);
                applyState(SuperstructureState.INTAKING);
                break;

            case SHOOTING:
                kicker.requestState(Kicker.SystemState.ENABLED);
                index.requestState(Index.SystemState.INDEXING);
                break;

            case SHOOTING_AND_INTAKING:
                applyState(SuperstructureState.SHOOTING);
                applyState(SuperstructureState.INTAKING);
                break;

            case STUCKED_RECOVERY:
                intake.requestState(Intake.SystemState.OUTTAKING);
                index.requestState(Index.SystemState.OUTTAKING);
                kicker.requestState(Kicker.SystemState.OUTTAKE);
                break;

            case TESTING:
                hood.requestState(Hood.SystemState.TESTING);
                turret.requestState(Turret.SystemState.TESTING);
                flywheel.requestState(Flywheel.SystemState.TESTING);
                index.requestState(Index.SystemState.TESTING);
                kicker.requestState(Kicker.SystemState.TESTING);
                break;

            case DEFAULT:
                turret.requestState(Turret.SystemState.TRACKING);
                break;

            case IDLE:
            default:
                stopAll();
                break;
        }
    }

    private void stopAll() {
        intake.requestState(Intake.SystemState.IDLE);
        index.requestState(Index.SystemState.IDLE);
        kicker.requestState(Kicker.SystemState.IDLE);
        hood.requestState(Hood.SystemState.IDLE);
        turret.requestState(Turret.SystemState.IDLE);
        flywheel.requestState(Flywheel.SystemState.IDLE);
        drive.stop();
    }

    /* ================= HELPERS ================= */

    public boolean isAnyShootingState(SuperstructureState state) {
        return state == SuperstructureState.PREP_SHOOTING
                || state == SuperstructureState.PREP_SHOOTING_AND_INTAKING
                || state == SuperstructureState.SHOOTING
                || state == SuperstructureState.SHOOTING_AND_INTAKING;
    }

    /* ================= COMMAND API ================= */

    /** Intake toggle */
    public Command intakeCommand() {
        return new InstantCommand(() -> {

            if (currentState == SuperstructureState.INTAKING || currentState == SuperstructureState.OPENING_INTAKE) {
                wantedState = SuperstructureState.CLOSING_AND_STOPPING_INTAKE;
            } else if (currentState == SuperstructureState.SHOOTING_AND_INTAKING) {
                wantedState = SuperstructureState.SHOOTING;
                applyState(SuperstructureState.CLOSING_AND_STOPPING_INTAKE);
            } else if (currentState == SuperstructureState.PREP_SHOOTING_AND_INTAKING) {
                wantedState = SuperstructureState.PREP_SHOOTING;
                applyState(SuperstructureState.CLOSING_AND_STOPPING_INTAKE);
            } else {
                wantedState = SuperstructureState.INTAKING;
            }

        }, this);
    }

    /** Shoot toggle (shoot + shoot&intake birleşik) */
    public Command shootCommand() {
        return new InstantCommand(() -> {

            // Eğer zaten shoot modundaysak → iptal
            if (isAnyShootingState(currentState)) {

                boolean intakeWasActive = currentState == SuperstructureState.PREP_SHOOTING_AND_INTAKING
                        || currentState == SuperstructureState.SHOOTING_AND_INTAKING;

                wantedState = intakeWasActive
                        ? SuperstructureState.INTAKING
                        : SuperstructureState.IDLE;

                return;
            }

            // Shoot başlat
            boolean intakeActive = currentState == SuperstructureState.INTAKING
                    || currentState == SuperstructureState.OPENING_INTAKE;

            wantedState = intakeActive
                    ? SuperstructureState.PREP_SHOOTING_AND_INTAKING
                    : SuperstructureState.PREP_SHOOTING;

        }, this);
    }

    /** Emergency stop */
    public Command stopCommand() {
        return new InstantCommand(() -> {
            wantedState = SuperstructureState.IDLE;
        }, this);
    }

    /** Test mode */
    public Command testingCommand() {
        return new InstantCommand(() -> {
            wantedState = SuperstructureState.TESTING;
        }, this);
    }

    @Logged
    public SuperstructureState getCurrentState() {
        return currentState;
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
}