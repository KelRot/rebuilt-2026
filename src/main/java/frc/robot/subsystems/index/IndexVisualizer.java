package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;


public class IndexVisualizer {

    /* ---- Constants ---- */
    private static final Distance INDEX_LENGTH = Inches.of(19.1); //These values may need to be adjusted later, they are random rn.
    private static final double ROOT_X = 0.75;
    private static final double ROOT_Y = 0.4;

    @AutoLogOutput
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 0.75);

    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d index;

    private double currentAngleDeg = 0.0;

    public IndexVisualizer(String name, Color color) {
        root = mechanism.getRoot(name + " Root", ROOT_X, ROOT_Y);

        index =
                root.append(
                        new LoggedMechanismLigament2d(
                                "Index",
                                INDEX_LENGTH,
                                Degrees.of(currentAngleDeg),
                                3,
                                new Color8Bit(color)));
    }

    /** Set intake angle in degrees */
    public void setAngleDeg(double angleDeg) {
        currentAngleDeg = angleDeg;
        index.setAngle(Degrees.of(angleDeg + 90.0));
    }

    /** Call EVERY cycle to log pose */
    public void update() {
        double angleRad = Degrees.of(currentAngleDeg).in(Radians);
        double lengthMeters = INDEX_LENGTH.in(Meters);

        double y = lengthMeters * Math.cos(angleRad);
        double z = lengthMeters * Math.sin(angleRad);

        Logger.recordOutput(
                "Index/Pose",
                new Pose3d(
                        0.0,
                        y,
                        z,
                        new Rotation3d(0.0, angleRad, 0.0)));
    }
}