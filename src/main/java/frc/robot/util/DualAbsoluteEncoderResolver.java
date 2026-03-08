package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class DualAbsoluteEncoderResolver {

  private final double ratio1;
  private final double ratio2;

  private final double minMechanismRot;
  private final double maxMechanismRot;

  private final double tolerance;

  public DualAbsoluteEncoderResolver(
      double ratio1,
      double ratio2,
      double minMechanismRot,
      double maxMechanismRot,
      double tolerance) {

    this.ratio1 = ratio1;
    this.ratio2 = ratio2;
    this.minMechanismRot = minMechanismRot;
    this.maxMechanismRot = maxMechanismRot;
    this.tolerance = tolerance;
  }

  /**
   * Calculates mechanism rotations from two absolute encoders.
   *
   * @param abs1 encoder1 absolute position (0–1 rotations)
   * @param abs2 encoder2 absolute position (0–1 rotations)
   * @return mechanism rotations, or NaN if not solvable
   */
  public double solve(double abs1, double abs2) {

    abs1 = MathUtil.inputModulus(abs1, 0.0, 1.0);
    abs2 = MathUtil.inputModulus(abs2, 0.0, 1.0);

    double bestError = Double.MAX_VALUE;
    double bestRotation = Double.NaN;

    double nMin = Math.min(ratio1 * minMechanismRot, ratio1 * maxMechanismRot) - abs1;
    double nMax = Math.max(ratio1 * minMechanismRot, ratio1 * maxMechanismRot) - abs1;

    int minN = (int) Math.floor(nMin) - 1;
    int maxN = (int) Math.ceil(nMax) + 1;

    for (int n = minN; n <= maxN; n++) {

      double mechRot = (abs1 + n) / ratio1;

      if (mechRot < minMechanismRot || mechRot > maxMechanismRot) {
        continue;
      }

      double predicted2 = MathUtil.inputModulus(ratio2 * mechRot, 0.0, 1.0);
      double error = modularError(predicted2, abs2);

      if (error < bestError) {
        bestError = error;
        bestRotation = mechRot;
      }
    }

    if (bestError > tolerance) {
      return Double.NaN;
    }

    return bestRotation;
  }

  private static double modularError(double a, double b) {
    double diff = Math.abs(a - b);
    return diff > 0.5 ? 1.0 - diff : diff;
  }
}