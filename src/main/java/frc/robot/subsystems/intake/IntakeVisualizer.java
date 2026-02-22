package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

public class IntakeVisualizer {

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d roller;
    private final String key;

    private static final double rollerLength = 0.4;
    private static final double rollerThickness = 10.0;

    private static final Translation3d intakePivotOffset = new Translation3d(
            -0.275, // X
            -0.300, // Y
            0.220 // Z
    );

    private static final double ZERO_ANGLE_RAD = Math.toRadians(0);

    public IntakeVisualizer(String key, Color color) {
        this.key = key;

        mechanism = new LoggedMechanism2d(2.0, 2.0, new Color8Bit(Color.kWhite));
        LoggedMechanismRoot2d root = mechanism.getRoot("intake", 1.0, 1.0);

        roller = new LoggedMechanismLigament2d(
                "roller",
                rollerLength,
                90 + 37.0,
                rollerThickness,
                new Color8Bit(color));

        root.append(roller);
    }

    public void update(double angleRads) {

        roller.setAngle(Rotation2d.fromRadians(angleRads));
        Logger.recordOutput("Intake/Mechanism2d/" + key, mechanism);

        double totalAngle = ZERO_ANGLE_RAD + angleRads;

        Pose3d intakePose = new Pose3d()

                .transformBy(
                        new Transform3d(
                                intakePivotOffset,
                                new Rotation3d()))

                .transformBy(
                        new Transform3d(
                                new Translation3d(),
                                new Rotation3d(0.0, totalAngle, 0.0)))

                .transformBy(
                        new Transform3d(
                                intakePivotOffset.unaryMinus(),
                                new Rotation3d()));

        Logger.recordOutput("Intake/Mechanism3d/" + key, intakePose);
    }
}