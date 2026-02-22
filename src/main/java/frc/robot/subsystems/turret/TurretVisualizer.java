package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

public class TurretVisualizer {

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d turretLigament;
  private final LoggedMechanismLigament2d targetLigament;
  private final String key;

  private static final double turretLength = 0.5;
  private static final double turretThickness = 5.0;

  // CAD’den gelen offsetler
  private static final Translation3d turretPosition =
      new Translation3d(0.135, 0.0, 0.262); // ✅ buraya CAD offsetleri ekledik

  private static final Rotation3d turretBaseRotation =
      new Rotation3d(
          Math.toRadians(0), // X
          0.0,                // Y
          Math.toRadians(0)   // Z
      );

  public TurretVisualizer(String key, Color color) {
    this.key = key;

    mechanism =
        new LoggedMechanism2d(2.0, 2.0, new Color8Bit(Color.kBlack));

    LoggedMechanismRoot2d root =
        mechanism.getRoot("turret", 1.0, 1.0);

    turretLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "turret",
                turretLength,
                0.0,
                turretThickness,
                new Color8Bit(color)));

    targetLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "target",
                turretLength,
                0.0,
                turretThickness / 2.0,
                new Color8Bit(Color.kLightSkyBlue)));
  }

  public void update(double angleRads, double setpointRads) {

    /* ================= 2D ================= */
    turretLigament.setAngle(Rotation2d.fromRadians(angleRads));
    targetLigament.setAngle(Rotation2d.fromRadians(setpointRads));

    Logger.recordOutput("Turret/Mechanism2d/" + key, mechanism);

    /* ================= 3D ================= */
    Pose3d basePose = new Pose3d(turretPosition, turretBaseRotation);

    // Turret pozisyonu ve rotasyonu
    Pose3d turretPose =
        basePose.transformBy(
            new Transform3d(
                new Translation3d(0, 0, 0),               // lokal offset yok
                new Rotation3d(0.0, 0.0, angleRads)      // ✅ sadece lokal Z
            ));

    Logger.recordOutput("Turret/Mechanism3d/" + key, turretPose);

    // Setpoint pozisyonu
    Pose3d setpointPose =
        basePose.transformBy(
            new Transform3d(
                new Translation3d(0, 0, 0),               // lokal offset yok
                new Rotation3d(0.0, 0.0, setpointRads)
            ));

    Logger.recordOutput("Turret/SetpointPose3d/" + key, setpointPose);
  }
}