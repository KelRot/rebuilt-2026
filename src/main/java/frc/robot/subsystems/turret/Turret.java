package frc.robot.subsystems.turret;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO io) {
        this.io = io;
    }

    public static enum SystemState {
        IDLE, TRACKING, SHOOTING, POSITION
    }

    private SystemState systemState = SystemState.IDLE;

    public double setpoint;

    public double calculateTurretRealPose() {
        double rot1 = inputs.absPositionTours1 / 360;
        double rot2 = inputs.absPositionTours2 / 360;

        double bestmatch = 0.0;
        double tolerance = 1e-4;

        for (int n1 = 0; n1 < TurretConstants.absEncoder2Teeth; n1++) {
            double candidate = (n1 + rot1) * TurretConstants.absEncoder1Teeth / TurretConstants.turretGearboxTeeth;

            for (int n2 = 0; n2 < TurretConstants.absEncoder1Teeth; n2++) {
                double candidate2 = (n2 + rot2) * TurretConstants.absEncoder2Teeth / TurretConstants.turretGearboxTeeth;

                if (Math.abs(candidate - candidate2) < tolerance) {
                    bestmatch = (candidate + candidate2) / 2;
                    return bestmatch;
                }
            }
        }

        return bestmatch;
    }

    public void motorPositionSet() {
        io.setPosition(calculateTurretRealPose());
        io.setEncoder(calculateTurretRealPose());
    }

    public void requestState(SystemState wantedState) {
        systemState = wantedState;
    }

    public void setPosition(double position) {
        requestState(SystemState.POSITION);
        setpoint = position;

    }

    public double angleToTarget(Translation2d target) {
        double dx = target.getX() - RobotContainer.getDrive().getPose().getX(); // botpose will be given later
        double dy = target.getY() - RobotContainer.getDrive().getPose().getY();

        double angleToTarget = Math.atan2(dy, dx);

        double turretAngle = angleToTarget - RobotContainer.getDrive().getPose().getRotation().getRadians();

        return MathUtil.angleModulus(turretAngle);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (systemState) {
            case IDLE:
                io.setVoltage(0);
                break;
            case TRACKING:
                setpoint = angleToTarget(); // target will be given later
                io.setPosition(setpoint);
                break;
            case SHOOTING:
                setpoint = angleToTarget(); // target will be given later
                io.setPosition(setpoint);
                break;
            case POSITION:
                io.setPosition(setpoint);
                break;
        }
    }

}
