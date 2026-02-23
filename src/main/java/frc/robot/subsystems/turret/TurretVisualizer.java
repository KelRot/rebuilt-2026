package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

public class TurretVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d turretLigament;
    private final LoggedMechanismLigament2d targetLigament;
    private final String key;

    private Translation3d[] trajectory = new Translation3d[50];
    private Supplier<Pose2d> poseSupplier = () -> RobotContainer.getDrive().getPose();
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier = () -> RobotContainer.getDrive().getFieldSpeeds();

    private static final double turretLength = 0.5;
    private static final double turretThickness = 5.0;

    // CAD’den gelen offsetler
    private static final Translation3d turretPosition = new Translation3d(0.135, 0.0, 0.262);

    private static final Rotation3d turretBaseRotation = new Rotation3d(
            Math.toRadians(0), // X
            0.0, // Y
            Math.toRadians(0) // Z
    );

    public TurretVisualizer(String key, Color color) {
        this.key = key;

        mechanism = new LoggedMechanism2d(2.0, 2.0, new Color8Bit(Color.kBlack));

        LoggedMechanismRoot2d root = mechanism.getRoot("turret", 1.0, 1.0);

        turretLigament = root.append(
                new LoggedMechanismLigament2d(
                        "turret",
                        turretLength,
                        0.0,
                        turretThickness,
                        new Color8Bit(color)));

        targetLigament = root.append(
                new LoggedMechanismLigament2d(
                        "target",
                        turretLength,
                        0.0,
                        turretThickness / 2.0,
                        new Color8Bit(Color.kLightSkyBlue)));
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = new Pose3d(
                poseSupplier.get().getX(), // x
                poseSupplier.get().getY(), // y
                0.0, // z = 0
                new Rotation3d(0.0, 0.0, poseSupplier.get().getRotation().getRadians()) // roll=0, pitch=0, yaw = 2D
                                                                                        // rotation
        );
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel = horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel = horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    public void updateFuel(LinearVelocity vel, Angle angle) {
        Translation3d trajVel = launchVel(vel, Degrees.of(90).minus(angle));
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t;

            trajectory[i] = new Translation3d(x, y, z);
        }

        Logger.recordOutput("Turret/Trajectory", trajectory);
    }

    public void update(double angleRads, double setpointRads) {

        turretLigament.setAngle(Rotation2d.fromRadians(angleRads));
        targetLigament.setAngle(Rotation2d.fromRadians(setpointRads));

        Logger.recordOutput("Turret/Mechanism2d/" + key, mechanism);

        Pose3d basePose = new Pose3d(turretPosition, turretBaseRotation);

        Pose3d turretPose = basePose.transformBy(
                new Transform3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0.0, 0.0, angleRads)));

        Logger.recordOutput("Turret/Mechanism3d/" + key, turretPose);

        // Setpoint pozisyonu
        Pose3d setpointPose = basePose.transformBy(
                new Transform3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0.0, 0.0, setpointRads)));

        Logger.recordOutput("Turret/SetpointPose3d/" + key, setpointPose);
    }
}