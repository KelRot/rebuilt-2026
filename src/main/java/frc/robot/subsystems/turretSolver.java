package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.DualAbsoluteEncoderResolver;

public class turretSolver extends SubsystemBase {
  public static DualAbsoluteEncoderResolver resolver = new DualAbsoluteEncoderResolver(
      24.0, // ratio1
      25.0, // ratio2
      -0.75, // minMechanismRot
      0.75, // maxMechanismRot
      0.005 // tolerance
  );
  private final DutyCycleEncoder absEncoder1;
  private final DutyCycleEncoder absEncoder2;

  public turretSolver() {
    absEncoder1 = new DutyCycleEncoder(TurretConstants.absEncoder1ID, 360.0, TurretConstants.abs1Offset);
    absEncoder2 = new DutyCycleEncoder(TurretConstants.absEncoder2ID, 360.0, TurretConstants.abs2Offset);
  }

  @Override
  public void periodic() {
    double abs1 = absEncoder1.get();
    double abs2 = absEncoder2.get();
    double turretRotations = resolver.solve(abs1, abs2);
    System.out.println("Turret rotations: " + turretRotations);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
