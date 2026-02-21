package frc.robot.subsystems.turret;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private double bestAngle = 0.0;
    private double minError = Double.MAX_VALUE;
    private final double minAngle = -180.0; //degree
    private final double maxAngle = 180.0; //degree
    private final double step = 0.5; //degree

    public Turret(TurretIO io) {
        this.io = io;
        motorPositionSet();
    }

    public static enum SystemState {
        IDLE, TRACKING, SHOOTING, POSITION
    }

    private SystemState systemState = SystemState.IDLE;

    public double manual_setpoint;
    public double hub_setpoint;

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
        manual_setpoint = position;

    }

    public double angleToTarget() {
        Translation2d target = RobotContainer.getVision().getBestHubTarget();
        double dx = target.getX() - RobotContainer.getDrive().getPose().getX(); // botpose will be given later
        double dy = target.getY() - RobotContainer.getDrive().getPose().getY();

        double angleToTarget = Math.atan2(dy, dx);

        double turretAngle = angleToTarget - RobotContainer.getDrive().getPose().getRotation().getRadians();

        return MathUtil.angleModulus(turretAngle);
    }

    @Override
    public void periodic() {
        hub_setpoint = angleToTarget();

        io.updateInputs(inputs);

        switch (systemState) {
            case IDLE:
                io.setVoltage(0);
                break;
            case TRACKING:
                io.setPosition(hub_setpoint);
                break;
            case SHOOTING:
                io.setPosition(hub_setpoint);
                break;
            case POSITION:
                io.setPosition(manual_setpoint);
                break;
        }
    }

}
