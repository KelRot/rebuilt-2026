public class Superstructure extends SubsystemBase {

    private final Index index;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Kicker kicker;
    private final Hood hood;
    private final Turret turret;
    private final Drive drive;

    private SuperstructureState currentState = SuperstructureState.IDLE;
    private SuperstructureState wantedState = SuperstructureState.IDLE;

    public enum SuperstructureState {
        OPENING_INTAKE,
        INTAKING,
        CLOSING_INTAKE,
        REJECTING_INTAKE,
        OUTTAKE,
        PREP_SHOOTING,
        SHOOTING,
        STOP_SHOOTING,
        STOP_INTAKING,
        DEFAULT,
        TESTING,
        STOP,
        IDLE
    }

    public Superstructure(
        Intake intake,
        Flywheel flywheel,
        Kicker kicker,
        Hood hood,
        Turret turret,
        Drive drive,
        Index index
    ) {
        this.intake = intake;
        this.flywheel = flywheel;
        this.kicker = kicker;
        this.hood = hood;
        this.turret = turret;
        this.drive = drive;
        this.index = index;
    }

    public void setWantedState(SuperstructureState state) {
        wantedState = state;
    }

    public SuperstructureState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {

        handleTransitions();
        runState(currentState);
    }

    /* ================= TRANSITIONS ================= */

    private void handleTransitions() {

        currentState = wantedState;

        if (currentState == SuperstructureState.PREP_SHOOTING
                && turret.isAtSetpoint()
                && hood.isAtSetpoint()
                && flywheel.isAtSetpoint()) {

            currentState = SuperstructureState.SHOOTING;
        }
    }

    /* ================= STATE ACTIONS ================= */

    private void runState(SuperstructureState state) {

        switch (state) {

            case OPENING_INTAKE:
                intake.requestState(Intake.SystemState.OPENING);
                break;

            case INTAKING:
                if (intake.isOpened()) {
                    intake.requestState(Intake.SystemState.INTAKING);
                    index.requestState(Index.SystemState.INDEXING);
                }
                break;

            case CLOSING_INTAKE:
                intake.requestState(Intake.SystemState.CLOSING);
                break;

            case REJECTING_INTAKE:
                if (intake.isOpened()) {
                    intake.requestState(Intake.SystemState.OUTTAKING);
                }
                break;

            case OUTTAKE:
                if (!intake.isOpened()) {
                    intake.requestState(Intake.SystemState.OPENING);
                }
                index.requestState(Index.SystemState.OUTTAKING);
                break;

            case PREP_SHOOTING:
                hood.requestState(Hood.SystemState.POSITION);
                turret.requestState(Turret.SystemState.POSITION);
                flywheel.requestState(Flywheel.SystemState.TARGET_RPM);
                break;

            case SHOOTING:
                kicker.requestState(Kicker.SystemState.ENABLED);
                break;

            case STOP_SHOOTING:
                kicker.requestState(Kicker.SystemState.IDLE);
                flywheel.requestState(Flywheel.SystemState.IDLE);
                wantedState = SuperstructureState.IDLE;
                break;

            case STOP_INTAKING:
                intake.requestState(Intake.SystemState.IDLE);
                wantedState = SuperstructureState.IDLE;
                break;

            case DEFAULT:
                defaultBehavior();
                break;

            case TESTING:
                testingBehavior();
                break;

            case STOP:
                stopAll();
                break;

            case IDLE:
            default:
                break;
        }
    }

    /* ================= BEHAVIORS ================= */

    private void defaultBehavior() {

        if (intake.isOpened()) {
            intake.requestState(Intake.SystemState.CLOSING);
        }

        index.requestState(Index.SystemState.IDLE);
        kicker.requestState(Kicker.SystemState.IDLE);
        turret.requestState(Turret.SystemState.TRACKING);
    }

    private void testingBehavior() {

        hood.requestState(Hood.SystemState.TESTING);
        turret.requestState(Turret.SystemState.TESTING);
        flywheel.requestState(Flywheel.SystemState.TESTING);
        index.requestState(Index.SystemState.TESTING);
        kicker.requestState(Kicker.SystemState.TESTING);
    }

    private void stopAll() {

        if (intake.isOpened()) {
            intake.requestState(Intake.SystemState.CLOSING);
        }

        index.stop();
        kicker.stop();
        hood.stop();
        turret.stop();
        flywheel.stop();
        drive.stop();
    }
}