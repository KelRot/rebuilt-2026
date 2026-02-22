
package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeVisualizer {

  /* ---------------- Constants ---------------- */

  // Intake kol uzunluğu (metre)
  private static final double INTAKE_LENGTH = 0.60;

  /* ---------------- Mechanism2d ---------------- */

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d intake;
  private final String key;
  public static final Translation2d armOrigin = new Translation2d(0, 0.0);

  public IntakeVisualizer(String key, Color color) {
    this.key = key;

    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);

    intake = new LoggedMechanismLigament2d(
        "intake",
        INTAKE_LENGTH,
        0.0,
        6,
        new Color8Bit(color));

    root.append(intake);

  }

  /**
   * @param angleRads Intake açısı (RADYAN)
   * @param robotPose Robotun field-relative Pose3d’si
   */
  public void update(double angleRad) {

    /* -------- Mechanism2d -------- */
    intake.setAngle(Rotation2d.fromDegrees(angleRad));
    Logger.recordOutput("Intake/Mechanism2d/" + key, mechanism);
    angleRad = Degrees.of(0).in(Radians);
    /* -------- 3D Pose (robota sabit) -------- */

    // Intake kendi pivotu etrafında X ekseni etrafında döner
    Transform3d intakeRotation = new Transform3d(
        0.0,
        0.0,
        0.0,
        new Rotation3d(-angleRad, 0.0, 0.0));

    Pose3d pivot = new Pose3d(armOrigin.getX(), 0.0, armOrigin.getY(), new Rotation3d(0.0, -angleRad, 0.0));

    Logger.recordOutput("Intake/Mechanism3d/" + key, pivot);
  }
}