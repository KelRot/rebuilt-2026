/*
 * ShotCalculator.java - Newton-method SOTM fire control with drag compensation
 *                       and drive heading alignment (no turret)
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 */

package frc.robot.util.shotutil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;

/**
 * Shoot-on-the-move fire control solver with drive heading alignment.
 *
 * <p>
 * Turret yok — robot'un kendisi hedefe döner. SOTM, drag compensation,
 * latency compensation aynen çalışır. calculate() çıktısındaki
 * {@code driveHeadingDeg} değerini doğrudan swerve heading PID'ine ver.
 *
 * <p>
 * Usage:
 *
 * <pre>
 * ShotCalculator calc = new ShotCalculator(config);
 * // LUT doldur...
 *
 * ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
 *     swerve.getPose(), swerve.getFieldVelocity(), swerve.getRobotVelocity(),
 *     visionConfidence);
 *
 * ShotCalculator.LaunchParameters result = calc.calculate(inputs, target);
 * if (result.isValid() && result.confidence() > 50) {
 *   shooter.setRPM(result.rpm());
 *   swerve.setHeadingTarget(result.driveHeadingDeg()); // field-frame derece
 * }
 * </pre>
 */
public class ShotCalculator {

  /**
   * calculate() sonucu.
   *
   * <ul>
   * <li>{@code rpm} – flywheel hedefi
   * <li>{@code timeOfFlightSec} – çözülen TOF (mechLatency dahil)
   * <li>{@code driveHeadingDeg} – field-frame robot yön hedefi [-180, 180)
   * Bu değeri swerve heading PID setpointi olarak kullan.
   * <li>{@code confidence} – 0-100 atış kalite skoru
   * </ul>
   */
  public record LaunchParameters(
      double rpm,
      double timeOfFlightSec,
      double driveHeadingDeg,
      boolean isValid,
      double confidence,
      double solvedDistanceM,
      int iterationsUsed,
      boolean warmStartUsed) {

    public static final LaunchParameters INVALID = new LaunchParameters(0, 0, 0, false, 0, 0, 0, false);
  }

  /**
   * Her cycle'da solver'a verilen robot durumu.
   */
  public record ShotInputs(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      ChassisSpeeds robotVelocity,
      double visionConfidence,
      double pitchDeg,
      double rollDeg) {

    public ShotInputs(
        Pose2d robotPose,
        ChassisSpeeds fieldVelocity,
        ChassisSpeeds robotVelocity,
        double visionConfidence) {
      this(robotPose, fieldVelocity, robotVelocity, visionConfidence, 0.0, 0.0);
    }
  }

  /**
   * Tuning parametreleri.
   */
  public static class Config {
    // Launcher geometrisi (CAD'den ölç)
    public double launcherOffsetX = 0.20; // robot merkezinden ileri (m)
    public double launcherOffsetY = 0.0; // robot merkezinden sola (m)

    // Geçerli atış mesafesi (m)
    public double minScoringDistance = 0.5;
    public double maxScoringDistance = 20.0;

    // Newton solver
    public int maxIterations = 25;
    public double convergenceTolerance = 0.001; // saniye
    public double tofMin = 0.05;
    public double tofMax = 5.0;

    // Bu hızın altında SOTM atla, direkt hedefe bak
    public double minSOTMSpeed = 0.1;

    // Bu hızın üzerinde atış reddet
    public double maxSOTMSpeed = 3.0;

    // Latency compensation
    public double phaseDelayMs = 15.0; // vision pipeline gecikmesi (ms)
    public double mechLatencyMs = 20.0; // mekanizma gecikmesi (ms)

    // Yatay hız sönümleme katsayısı (1/s)
    // displacement = v0 * (1 - e^(-c*t)) / c
    // ~0.24 tipik FRC topu için. 0 = devre dışı.
    public double sotmDragCoeff = 0.24;

    // Heading toleransı için confidence scoring (derece)
    public double headingMaxErrorDeg = 5.0;

    // Hız arttıkça tolerans daralır: scaledMaxError = base / (1 + scalar * speed)
    public double headingSpeedScalar = 1.0;

    // Uzaklık arttıkça tolerans daralır
    public double headingReferenceDistance = 2.5; // metre

    // Tilt gate: eğim bu değeri geçerse atış reddedilir (derece). 90 = kapalı.
    public double maxTiltDeg = 5.0;

   

    // Confidence scoring ağırlıkları
    public double wConvergence = 1.0;
    public double wVelocityStability = 0.8;
    public double wVisionConfidence = 1.2;
    public double wHeadingAccuracy = 1.5;
    public double wDistanceInRange = 0.5;
  }
  

  private final Config config;

 private double shootConfidence;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

  private ShotLUT shotLUT = null;
  private double rpmOffset = 0;

  // Warm-start
  private double previousTOF = -1;
  private double previousSpeed = 0;

  // Hızlanma tahmini için önceki cycle değerleri
  private double prevRobotVx = 0;
  private double prevRobotVy = 0;
  private double prevRobotOmega = 0;

  public ShotCalculator(Config config) {
    this.config = config;
  }

  public ShotCalculator() {
    this(new Config());
  }

  // -------------------------------------------------------------------------
  // LUT
  // -------------------------------------------------------------------------

  public void loadLUTEntry(double distanceM, double rpm, double tof) {
    rpmMap.put(distanceM, rpm);
    tofMap.put(distanceM, tof);
  }

  double effectiveRPM(double distance) {
    double base = shotLUT != null ? shotLUT.getRPM(distance) : rpmMap.get(distance);
    Double correction = correctionRpmMap.get(distance);
    return base + (correction != null ? correction : 0.0) + rpmOffset;
  }

  double effectiveTOF(double distance) {
    double base = shotLUT != null ? shotLUT.getTOF(distance) : tofMap.get(distance);
    Double correction = correctionTofMap.get(distance);
    return base + (correction != null ? correction : 0.0);
  }

  // -------------------------------------------------------------------------
  // Physics
  // -------------------------------------------------------------------------

  private double dragCompensatedTOF(double tof) {
    double c = config.sotmDragCoeff;
    if (c < 1e-6)
      return tof;
    return (1.0 - Math.exp(-c * tof)) / c;
  }

  private static final double DERIV_H = 0.01;

  double tofMapDerivative(double d) {
    return (effectiveTOF(d + DERIV_H) - effectiveTOF(d - DERIV_H)) / (2.0 * DERIV_H);
  }

  // -------------------------------------------------------------------------
  // Ana solver
  // -------------------------------------------------------------------------

  /**
   * Her cycle'da bir kez çağır.
   *
   * @param inputs robot durumu
   * @param target field-frame hedef koordinatı
   * @return atış çözümü, ya da {@link LaunchParameters#INVALID}
   */
  public LaunchParameters calculate(ShotInputs inputs, Translation2d target) {
    if (inputs == null || inputs.robotPose() == null
        || inputs.fieldVelocity() == null || inputs.robotVelocity() == null
        || target == null) {
      return LaunchParameters.INVALID;
    }

    Pose2d rawPose = inputs.robotPose();
    ChassisSpeeds fieldVel = inputs.fieldVelocity();
    ChassisSpeeds robotVel = inputs.robotVelocity();

    double poseX = rawPose.getX();
    double poseY = rawPose.getY();
    if (Double.isNaN(poseX) || Double.isNaN(poseY)
        || Double.isInfinite(poseX) || Double.isInfinite(poseY)) {
      return LaunchParameters.INVALID;
    }

    // Tilt gate
    if (Math.abs(inputs.pitchDeg()) > config.maxTiltDeg
        || Math.abs(inputs.rollDeg()) > config.maxTiltDeg) {
      return LaunchParameters.INVALID;
    }

    // -----------------------------------------------------------------------
    // 2. dereceden latency compensated pose tahmini
    // -----------------------------------------------------------------------
    double dt = config.phaseDelayMs / 1000.0;
    double ax = (robotVel.vxMetersPerSecond - prevRobotVx) / 0.02;
    double ay = (robotVel.vyMetersPerSecond - prevRobotVy) / 0.02;
    double aOmega = (robotVel.omegaRadiansPerSecond - prevRobotOmega) / 0.02;

    Pose2d compensatedPose = rawPose.exp(new Twist2d(
        robotVel.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
        robotVel.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
        robotVel.omegaRadiansPerSecond * dt + 0.5 * aOmega * dt * dt));

    prevRobotVx = robotVel.vxMetersPerSecond;
    prevRobotVy = robotVel.vyMetersPerSecond;
    prevRobotOmega = robotVel.omegaRadiansPerSecond;

    double robotX = compensatedPose.getX();
    double robotY = compensatedPose.getY();
    double heading = compensatedPose.getRotation().getRadians();

    double targetX = target.getX();
    double targetY = target.getY();

    // -----------------------------------------------------------------------
    // Launcher pozisyonu ve hızı
    // -----------------------------------------------------------------------
    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);

    double launcherX = robotX + config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherY = robotY + config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;

    double launcherFieldOffX = config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherFieldOffY = config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;
    double omega = fieldVel.omegaRadiansPerSecond;
    double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
    double vy = fieldVel.vyMetersPerSecond + (launcherFieldOffX) * omega;

    // -----------------------------------------------------------------------
    // Launcher → hedef vektörü
    // -----------------------------------------------------------------------
    double rx = targetX - launcherX;
    double ry = targetY - launcherY;
    double distance = Math.hypot(rx, ry);

    if (distance < config.minScoringDistance || distance > config.maxScoringDistance) {
      return LaunchParameters.INVALID;
    }

    double robotSpeed = Math.hypot(vx, vy);
    if (robotSpeed > config.maxSOTMSpeed) {
      return LaunchParameters.INVALID;
    }

    boolean velocityFiltered = robotSpeed < config.minSOTMSpeed;

    // -----------------------------------------------------------------------
    // SOTM Newton solver (veya statik fallback)
    // -----------------------------------------------------------------------
    double solvedTOF;
    double projDist;
    int iterationsUsed;
    boolean warmStartUsed;

    if (velocityFiltered) {
      solvedTOF = effectiveTOF(distance);
      projDist = distance;
      iterationsUsed = 0;
      warmStartUsed = false;
    } else {
      double tof;
      if (previousTOF > 0) {
        tof = previousTOF;
        warmStartUsed = true;
      } else {
        tof = effectiveTOF(distance);
        warmStartUsed = false;
      }

      projDist = distance;
      iterationsUsed = 0;

      for (int i = 0; i < config.maxIterations; i++) {
        double prevTOF = tof;

        double c = config.sotmDragCoeff;
        double dragExp = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
        double driftT = c < 1e-6 ? tof : (1.0 - dragExp) / c;

        double prx = rx - vx * driftT;
        double pry = ry - vy * driftT;
        projDist = Math.hypot(prx, pry);

        if (projDist < 0.01) {
          tof = effectiveTOF(distance);
          iterationsUsed = config.maxIterations + 1;
          break;
        }

        double lookupTOF = effectiveTOF(projDist);
        double dPrime = -dragExp * (prx * vx + pry * vy) / projDist;
        double gPrime = tofMapDerivative(projDist);
        double f = lookupTOF - tof;
        double fPrime = gPrime * dPrime - 1.0;

        tof = Math.abs(fPrime) > 0.01 ? tof - f / fPrime : lookupTOF;
        tof = MathUtil.clamp(tof, config.tofMin, config.tofMax);

        iterationsUsed = i + 1;
        if (Math.abs(tof - prevTOF) < config.convergenceTolerance)
          break;
      }

      if (tof > config.tofMax || tof < 0.0 || Double.isNaN(tof)) {
        tof = effectiveTOF(distance);
        iterationsUsed = config.maxIterations + 1;
      }

      solvedTOF = tof;
    }

    previousTOF = solvedTOF;

    double effectiveTOFValue = solvedTOF + config.mechLatencyMs / 1000.0;
    double effectiveRPMValue = effectiveRPM(projDist);

    // -----------------------------------------------------------------------
    // Velocity-compensated aim point → drive heading
    // -----------------------------------------------------------------------
    // Velocity-compensated aim point → drive heading
    double compTargetX, compTargetY;
    if (velocityFiltered) {
      compTargetX = targetX;
      compTargetY = targetY;
    } else {
      double headingDriftTOF = dragCompensatedTOF(solvedTOF);
      compTargetX = targetX - vx * headingDriftTOF;
      compTargetY = targetY - vy * headingDriftTOF;
    }

    // Launcher pozisyonundan hedef açısı, sonra -180° offset
    double aimDx = compTargetX - launcherX;
    double aimDy = compTargetY - launcherY;
    double driveHeadingDeg = Math.toDegrees(Math.atan2(aimDy, aimDx)) +180 ; //CHECK THIS MIGHT BE POSITIVE OR 0

    // [-180, 180) aralığına normalize et
    driveHeadingDeg = ((driveHeadingDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

    // -----------------------------------------------------------------------
    // Confidence için heading error (SOTM sapması)
    // -----------------------------------------------------------------------
    double headingErrorDeg;
    if (velocityFiltered) {
      headingErrorDeg = 0;
    } else {
      double staticAngleDeg = Math.toDegrees(Math.atan2(ry, rx));
      double diff = driveHeadingDeg - staticAngleDeg;
      diff = ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
      headingErrorDeg = Math.abs(diff);
    }

    // -----------------------------------------------------------------------
    // Solver convergence kalitesi
    // -----------------------------------------------------------------------
    double solverQuality;
    if (velocityFiltered) {
      solverQuality = 1.0;
    } else {
      int maxIter = config.maxIterations;
      if (iterationsUsed > maxIter) {
        solverQuality = 0.0;
      } else if (iterationsUsed <= 3) {
        solverQuality = 1.0;
      } else {
        solverQuality = MathUtil.interpolate(1.0, 0.1,
            (double) (iterationsUsed - 3) / (maxIter - 3));
      }
    }

    double confidence = computeConfidence(
        solverQuality, robotSpeed, headingErrorDeg, distance, inputs.visionConfidence());

    previousSpeed = robotSpeed;

    return new LaunchParameters(
        effectiveRPMValue,
        effectiveTOFValue,
        driveHeadingDeg,
        true,
        confidence,
        distance,
        iterationsUsed,
        warmStartUsed);
  }

  // -------------------------------------------------------------------------
  // Confidence scoring
  // -------------------------------------------------------------------------
  
  private double computeConfidence(
      double solverQuality, double currentSpeed, double headingErrorDeg,
      double distance, double visionConfidence) {

    double convergenceQuality = solverQuality;

    double speedDelta = Math.abs(currentSpeed - previousSpeed);
    double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);

    double visionConf = MathUtil.clamp(visionConfidence, 0, 1);

    double distanceScale = MathUtil.clamp(config.headingReferenceDistance / distance, 0.5, 2.0);
    double speedScale = 1.0 / (1.0 + config.headingSpeedScalar * currentSpeed);
    double scaledMaxError = config.headingMaxErrorDeg * distanceScale * speedScale;
    double headingAccuracy = MathUtil.clamp(1.0 - headingErrorDeg / scaledMaxError, 0, 1);

    double rangeSpan = config.maxScoringDistance - config.minScoringDistance;
    double rangeFraction = (distance - config.minScoringDistance) / rangeSpan;
    double distInRange = MathUtil.clamp(1.0 - 2.0 * Math.abs(rangeFraction - 0.5), 0, 1);

    double[] c = { convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange };
    double[] w = {
        config.wConvergence,
        config.wVelocityStability,
        config.wVisionConfidence,
        config.wHeadingAccuracy,
        config.wDistanceInRange
    };

    double sumW = 0, logSum = 0;
    for (int i = 0; i < 5; i++) {
      if (c[i] <= 0)
        return 0;
      logSum += w[i] * Math.log(c[i]);
      sumW += w[i];
    }
    shootConfidence = sumW <= 0 ? 0 : MathUtil.clamp(Math.exp(logSum / sumW) * 100.0, 0, 100);
    return shootConfidence;
  }

  // -------------------------------------------------------------------------
  // Correction / offset API
  // -------------------------------------------------------------------------

  public void addRpmCorrection(double distance, double deltaRpm) {
    correctionRpmMap.put(distance, deltaRpm);
  }

  public void addTofCorrection(double distance, double deltaTof) {
    correctionTofMap.put(distance, deltaTof);
  }

  public void clearCorrections() {
    correctionRpmMap.clear();
    correctionTofMap.clear();
  }

  /** Copilot D-pad RPM trim. ±200 RPM ile sınırlı. */
  public void adjustOffset(double delta) {
    rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
  }

  public void resetOffset() {
    rpmOffset = 0;
  }

  public double getOffset() {
    return rpmOffset;
  }

  // -------------------------------------------------------------------------
  // Accessors
  // -------------------------------------------------------------------------

  public double getTimeOfFlight(double distanceM) {
    return effectiveTOF(distanceM);
  }

  public double getBaseRPM(double distance) {
    if (shotLUT != null)
      return shotLUT.getRPM(distance);
    return rpmMap.get(distance);
  }

  public void resetWarmStart() {
    previousTOF = -1;
    previousSpeed = 0;
    prevRobotVx = 0;
    prevRobotVy = 0;
    prevRobotOmega = 0;
  }

  public void loadShotLUT(ShotLUT lut) {
    this.shotLUT = lut;
  }

  public double getHoodAngle(double distance) {
    if (shotLUT != null)
      return shotLUT.getAngle(distance);
    return 0;
  }

  // -------------------------------------------------------------------------
  // Test helpers
  // -------------------------------------------------------------------------

  InterpolatingDoubleTreeMap getRpmMap() {
    return rpmMap;
  }

  InterpolatingDoubleTreeMap getTofMap() {
    return tofMap;
  }
  public void periodic() {
    // Debug output
    SmartDashboard.putNumber("ShotCalc/LastConfidence", shootConfidence);}
  }