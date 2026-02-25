package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive.RobotZone;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.rebuilt.field.Field;
import frc.robot.util.rebuilt.field.FieldHelpers;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretVisualizer visualizer = new TurretVisualizer("Measured", Color.kRed);

    private double bestAngle = 0.0;
    private final double minAngle = -270.0; // degree
    private final double maxAngle = 270.0; // degree
    private final double CAPACITY = 10;
    private double fuelStored = 0;

    public Turret(TurretIO io) {
        this.io = io;
        motorPositionSet();
    }

    public static enum SystemState {
        IDLE, TRACKING, SHOOTING, POSITION, TESTING
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

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    // called repeatedly
    public void launchFuel() {
        if (fuelStored == 0)
            return;
        fuelStored--;

        TurretIOSim.getFuelSim().launchFuel(
                LinearVelocity.ofBaseUnits(14, MetersPerSecond),
                Degrees.of(60),
                Degrees.of(angleToTarget()),
                Distance.ofBaseUnits(0.262, Meters));
    }
// calculate this later 
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

   public Translation2d getTarget() {
        RobotZone robotPose = RobotContainer.getDrive().getRobotZone();
    switch (robotPose) {
        case UPPER_NEUTRAL_ZONE -> {
            return new Translation2d(FieldHelpers.flipXifRed(2.510), 6); // Sıfır koordinat
        }
        case LOWER_NEUTRAL_ZONE -> {
            return new Translation2d(FieldHelpers.flipXifRed(2.510), 2); // Sıfır koordinat
        }
        case BLUE_ALLIANCE_ZONE -> {
            if(Field.isRed()) {
                if(RobotContainer.getDrive().getPose().getY() < Field.getBlueHubCenter().getY()) {
                    return new Translation2d(FieldHelpers.flipXifRed(2.510), 2); // Sıfır koordinat
                } else {
                    return new Translation2d(FieldHelpers.flipXifRed(2.510), 6); // Sıfır koordinat
                }
            }
            return Field.getBlueHubCenter().toTranslation2d();// Blue hub pozisyonu
        }
        case RED_ALLIANCE_ZONE -> {
            if(Field.isBlue()) {
                if(RobotContainer.getDrive().getPose().getY() < Field.getRedHubCenter().getY()) {
                    return new Translation2d(FieldHelpers.flipXifRed(2.510), 2); // Sıfır koordinat
                } else {
                    return new Translation2d(FieldHelpers.flipXifRed(2.510), 6); // Sıfır koordinat
                }
            }
            return Field.getRedHubCenter().toTranslation2d();// Red hub pozisyonu
        }
        default -> {
            return new Translation2d(0.0, 0.0); // Default target
        }
    }
}

   public double angleToTarget() {
    double minError = Double.MAX_VALUE;

    Pose2d robotPose = RobotContainer.getDrive().getPose();
    Translation2d target = getTarget();

    // Turret pozisyonunu robot merkezine göre göreli hale getir
    Translation2d turretRelativeToRobot = TurretConstants.turretOffset; // zaten robot relative offset
    // robotPose.getRotation() ile field'a çevirmeye gerek yok, robot ekseninde zaten relative

    double dx = target.getX() - (robotPose.getX() + turretRelativeToRobot.getX());
    double dy = target.getY() - (robotPose.getY() + turretRelativeToRobot.getY());

    double targetRad = Math.atan2(dy, dx);

    // Turret relative açı: robot heading'inden çıkar
    double turretRad = targetRad - robotPose.getRotation().getRadians();

    // wrap between -π..π
    turretRad = FieldHelpers.normalizeAngle(turretRad);

    // Sınırlar içinde kalacak şekilde
    if (turretRad < Math.toRadians(minAngle)) turretRad += 2 * Math.PI;
    if (turretRad > Math.toRadians(maxAngle)) turretRad -= 2 * Math.PI;

    bestAngle = turretRad;
    return bestAngle;
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
            case TESTING:
                io.setVoltage(1.0);
                break;
        }
        
        inputs.isAtSetpoint = io.isAtSetpoint();
        visualizer.update(inputs.positionRads, hub_setpoint);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/SystemState", systemState.toString());
        Logger.recordOutput("Turret/Target", getTarget());
    }

    @Override
    public void simulationPeriodic() {
        visualizer.updateFuel(LinearVelocity.ofBaseUnits(4.0, MetersPerSecond), Degrees.of(60));
    }
    public boolean isAtSetpoint(){
        return io.isAtSetpoint();
    }
    public void stop() {
        io.stop();
    }
}
