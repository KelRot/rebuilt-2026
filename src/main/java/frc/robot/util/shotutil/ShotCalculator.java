/*
 * ShotCalculator.java - Newton-method SOTM fire control with drag compensation
 *                       and predictive turret setpoint (replaces FF approach)
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

package frc.robot.util.shotutil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Shoot-on-the-move fire control solver with predictive turret setpoint output.
 *
 * <p>
 * Figures out what RPM and turret angle your robot needs while driving.
 * Accounts for robot velocity, launcher offset, processing latency, and drag.
 *
 * <p>
 * Core SOTM idea: if you're moving, you can't just aim at the target because
 * the ball inherits your velocity. Newton's method finds the self-consistent
 * time-of-flight where the projected aim point and the LUT-predicted TOF agree.
 * Usually converges in 2-3 iterations.
 *
 * <p>
 * <b>Predictive setpoint vs FF:</b> Instead of computing a turret angular
 * velocity feedforward (which fights the position PID), we predict where the
 * turret setpoint will need to be after {@code mechLatencyMs} and output that
 * as the setpoint directly. The position controller tracks it cleanly without
 * any conflict. The prediction accounts for both target tracking rate and
 * robot rotation (which shifts the robot-relative angle at -omega).
 *
 * <p>
 * The target is passed per-call so callers can switch targets dynamically
 * (e.g. based on RobotZone) without reconfiguring the calculator.
 *
 * <p>
 * Turret angle is robot-relative, in degrees, in the [0, 360) range.
 * A shooterAngleOffsetDeg of -90 means the turret faces right by default.
 *
 * <p>
 * Usage:
 * 
 * <pre>
 * ShotCalculator.Config config = new ShotCalculator.Config();
 * config.launcherOffsetX = 0.23;
 * config.shooterAngleOffsetDeg = -90.0;
 * config.mechLatencyMs = 20.0;
 *
 * ShotCalculator calc = new ShotCalculator(config);
 *
 * for (var entry : lut.entries()) {
 *   if (entry.reachable()) {
 *     calc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
 *   }
 * }
 *
 * // call once per robot cycle
 * ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
 *     swerve.getPose(), swerve.getFieldVelocity(), swerve.getRobotVelocity(),
 *     visionConfidence);
 * Translation2d target = getTarget();
 * ShotCalculator.LaunchParameters result = calc.calculate(inputs, target);
 * if (result.isValid() &amp;&amp; result.confidence() &gt; 50) {
 *   shooter.setRPM(result.rpm());
 *   // Use predictiveTurretAngleDeg as the position setpoint — no FF needed.
 *   turret.setAngle(result.predictiveTurretAngleDeg());
 * }
 * </pre>
 */
public class ShotCalculator {

  /**
   * The result of calculate().
   *
   * <ul>
   * <li>{@code rpm} – flywheel target
   * <li>{@code timeOfFlightSec} – solved TOF including mechLatency
   * <li>{@code turretAngleDeg} – instantaneous robot-relative aim angle [0, 360)
   * <li>{@code predictiveTurretAngleDeg} – where the turret <em>needs to be</em>
   * after mechLatencyMs; use this as your position setpoint instead of
   * turretAngleDeg — no FF required
   * <li>{@code confidence} – 0-100 shot quality score
   * </ul>
   */
  public record LaunchParameters(
      double rpm,
      double timeOfFlightSec,
      double turretAngleDeg,
      double predictiveTurretAngleDeg,
      boolean isValid,
      double confidence,
      double solvedDistanceM,
      int iterationsUsed,
      boolean warmStartUsed) {

    public static final LaunchParameters INVALID = new LaunchParameters(0, 0, 0, 0, false, 0, 0, 0, false);
  }

  /**
   * All the state the solver needs from your robot each cycle.
   * Target is NOT included here — pass it directly to calculate().
   *
   * <p>
   * pitchDeg and rollDeg are absolute tilt angles in degrees. If your gyro
   * doesn't report these, use the 4-argument constructor.
   */
  public record ShotInputs(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      ChassisSpeeds robotVelocity,
      double visionConfidence,
      double pitchDeg,
      double rollDeg) {

    /** Convenience constructor for callers without pitch/roll data. */
    public ShotInputs(
        Pose2d robotPose,
        ChassisSpeeds fieldVelocity,
        ChassisSpeeds robotVelocity,
        double visionConfidence) {
      this(robotPose, fieldVelocity, robotVelocity, visionConfidence, 0.0, 0.0);
    }
  }

  /**
   * Tuning parameters. Wire to SmartDashboard/TunableNumber for live adjustment.
   */
  public static class Config {
    // Launcher geometry (measure from CAD)
    public double launcherOffsetX = 0.20; // meters forward of robot center
    public double launcherOffsetY = 0.0; // meters left of robot center

    // Scoring distance bounds (meters)
    public double minScoringDistance = 0.5;
    public double maxScoringDistance = 20;

    // Newton solver tuning
    public int maxIterations = 25;
    public double convergenceTolerance = 0.001; // seconds
    public double tofMin = 0.05;
    public double tofMax = 5.0;

    // Below this speed (m/s), skip SOTM and aim straight at the target
    public double minSOTMSpeed = 0.1;

    // Above this speed (m/s), refuse to shoot — outside calibration range
    public double maxSOTMSpeed = 3.0;

    // Latency compensation
    public double phaseDelayMs = 15.0; // vision pipeline lag (ms)
    public double mechLatencyMs = 20.0; // mechanism response latency (ms)
                                        // also used as predictive setpoint look-ahead

    /**
     * Predictive setpoint look-ahead scale factor.
     *
     * <p>
     * The look-ahead time is
     * {@code mechLatencyMs * predictiveLookAheadScale / 1000}.
     * Start at 0.5 and increase toward 1.0 as you verify the turret doesn't
     * overshoot.
     * Set to 0 to disable prediction and use the instantaneous angle only.
     */
    public double predictiveLookAheadScale = 0.5;

    // Linear drag damping constant (1/s) for SOTM horizontal velocity decay.
    // displacement = v0 * (1 - e^(-c*t)) / c
    // ~0.24 for a typical FRC ball at ~10 m/s. Set to 0 to disable.
    public double sotmDragCoeff = 0.24;

    // Turret physical limits (degrees, 0-360)
    public double turretMinAngleDeg = 0.0;
    public double turretMaxAngleDeg = 360.0;

    // Rotation between the turret zero and robot front (degrees).
    // -90 = turret faces right at 0°, 0 = forward, 180 = rear.
    public double shooterAngleOffsetDeg = -90.0;

    // Turret angle tolerance for confidence scoring (degrees).
    public double turretMaxErrorDeg = 5.0;

    // Heading tolerance tightens as robot speed increases.
    // scaledMaxError = base / (1 + speedScalar * speed). Set 0 to disable.
    public double headingSpeedScalar = 1.0;

    // Heading tolerance scales with distance (farther = tighter, same angle →
    // larger miss).
    // scaledMaxError *= referenceDistance / distance, clamped [0.5, 2.0].
    public double headingReferenceDistance = 2.5; // meters

    // Suppress firing when pitch or roll exceeds this (degrees). Set 90 to disable.
    public double maxTiltDeg = 5.0;

    // Confidence scoring weights (5-component weighted geometric mean)
    public double wConvergence = 1.0;
    public double wVelocityStability = 0.8;
    public double wVisionConfidence = 1.2;
    public double wHeadingAccuracy = 1.5;
    public double wDistanceInRange = 0.5;
  }

  private final Config config;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

  // If set via loadShotLUT(), base RPM/TOF/angle come from here instead of
  // the separate maps. Corrections and copilot offset still layer on top.
  private ShotLUT shotLUT = null;

  // Copilot RPM trim (flat offset applied during match)
  private double rpmOffset = 0;

  // Warm-start state
  private double previousTOF = -1;
  private double previousSpeed = 0;

  // Previous-cycle velocities for acceleration estimation
  private double prevRobotVx = 0;
  private double prevRobotVy = 0;
  private double prevRobotOmega = 0;

  public ShotCalculator(Config config) {
    this.config = config;
  }

  /** Default config. Call loadLUTEntry() to fill the lookup tables. */
  public ShotCalculator() {
    this(new Config());
  }

  // -------------------------------------------------------------------------
  // LUT population
  // -------------------------------------------------------------------------

  /**
   * Add a distance/RPM/TOF calibration point. Use ProjectileSimulator or
   * hand-tune.
   */
  public void loadLUTEntry(double distanceM, double rpm, double tof) {
    rpmMap.put(distanceM, rpm);
    tofMap.put(distanceM, tof);
  }

  // -------------------------------------------------------------------------
  // LUT lookups (base + corrections + offset)
  // -------------------------------------------------------------------------

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
  // Physics helpers
  // -------------------------------------------------------------------------

  /**
   * Drag-adjusted displacement factor. The ball's horizontal speed decays as
   * v(t) = v0 * e^(-ct), so displacement = v0 * (1 - e^(-ct)) / c.
   * Returns the effective time multiplier: (1 - e^(-c*tof)) / c.
   * When drag is disabled (c ≈ 0) this reduces to plain tof.
   */
  private double dragCompensatedTOF(double tof) {
    double c = config.sotmDragCoeff;
    if (c < 1e-6)
      return tof;
    return (1.0 - Math.exp(-c * tof)) / c;
  }

  /** Central finite-difference derivative of the TOF lookup table (1 cm step). */
  private static final double DERIV_H = 0.01;

  double tofMapDerivative(double d) {
    return (effectiveTOF(d + DERIV_H) - effectiveTOF(d - DERIV_H)) / (2.0 * DERIV_H);
  }

  // -------------------------------------------------------------------------
  // Angle helpers
  // -------------------------------------------------------------------------

  /**
   * Convert a field-frame aim angle to a robot-relative turret angle in [0, 360).
   */
  private double toRobotRelative(double fieldAimAngleDeg, double headingRad) {
    double angle = fieldAimAngleDeg - Math.toDegrees(headingRad) + config.shooterAngleOffsetDeg;
    return ((angle % 360.0) + 360.0) % 360.0;
  }

  // -------------------------------------------------------------------------
  // Main solver
  // -------------------------------------------------------------------------

  /**
   * Solve for the firing solution. Call once per cycle in robotPeriodic().
   *
   * <p>
   * Pass the target directly here so you can switch targets per-cycle
   * (e.g. from a zone-based getTarget()) without rebuilding ShotInputs.
   *
   * @param inputs robot state this cycle
   * @param target field-frame position to aim at
   * @return firing solution, or {@link LaunchParameters#INVALID} if not possible
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

    // Tilt gate: bumps/ramps tilt the launcher off-axis
    if (Math.abs(inputs.pitchDeg()) > config.maxTiltDeg
        || Math.abs(inputs.rollDeg()) > config.maxTiltDeg) {
      return LaunchParameters.INVALID;
    }

    // -----------------------------------------------------------------------
    // Second-order latency-compensated pose prediction
    // v*dt + 0.5*a*dt^2 tracks better through turns than first-order
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
    // Launcher position & velocity (includes rotational contribution)
    // -----------------------------------------------------------------------
    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);

    double launcherX = robotX + config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherY = robotY + config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;

    // v_launcher = v_robot + omega × r_launcher
    double launcherFieldOffX = config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherFieldOffY = config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;
    double omega = fieldVel.omegaRadiansPerSecond;
    double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
    double vy = fieldVel.vyMetersPerSecond + (launcherFieldOffX) * omega;

    // -----------------------------------------------------------------------
    // Launcher-to-target displacement
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
    // SOTM Newton solver (or static fallback)
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
    // Velocity-compensated aim point
    // -----------------------------------------------------------------------
    double compTargetX, compTargetY;
    if (velocityFiltered) {
      compTargetX = targetX;
      compTargetY = targetY;
    } else {
      double headingDriftTOF = dragCompensatedTOF(solvedTOF);
      compTargetX = targetX - vx * headingDriftTOF;
      compTargetY = targetY - vy * headingDriftTOF;
    }

    // -----------------------------------------------------------------------
    // Instantaneous turret angle (robot-relative, [0, 360))
    // -----------------------------------------------------------------------
    double aimX = compTargetX - robotX;
    double aimY = compTargetY - robotY;
    double fieldAimAngleDeg = Math.toDegrees(Math.atan2(aimY, aimX));
    double turretAngleDeg = toRobotRelative(fieldAimAngleDeg, heading);

    if (turretAngleDeg < config.turretMinAngleDeg
        || turretAngleDeg > config.turretMaxAngleDeg) {
      return LaunchParameters.INVALID;
    }

    // -----------------------------------------------------------------------
    // Predictive setpoint
    //
    // Why not FF? When using position PID, adding a voltage feedforward fights
    // the controller. Instead we predict where the setpoint will need to be
    // after mechLatencyMs and hand that directly to the position controller.
    //
    // turret angular rate = target tracking rate − robot rotation rate
    // • tracking rate: d/dt of (field aim angle) projected onto the turret
    // ≈ tangential velocity / distance (cross product / r^2)
    // • robot rotation: omega shifts the robot-relative angle at −omega
    //
    // lookAheadSec = mechLatencyMs * predictiveLookAheadScale / 1000
    // Start with predictiveLookAheadScale = 0.5; increase toward 1.0 once
    // you confirm the turret doesn't overshoot during fast turns.
    // -----------------------------------------------------------------------
    double lookAheadSec = config.mechLatencyMs * config.predictiveLookAheadScale / 1000.0;

    double predictiveTurretAngleDeg;
    if (lookAheadSec < 1e-6 || velocityFiltered) {
      // No prediction requested, or robot is nearly stationary — use instant angle.
      predictiveTurretAngleDeg = turretAngleDeg;
    } else {
      // Tracking rate: how fast the required field-frame aim angle is changing.
      // = (rx * vy − ry * vx) / distance^2 [rad/s], converted to deg/s
      // (cross product of displacement and relative velocity, divided by |r|^2)
      double trackingRateDegPerSec = Math.toDegrees(
          (rx * vy - ry * vx) / (distance * distance));

      // Robot rotation shifts the robot-relative turret angle at −omega.
      double robotOmegaDegPerSec = Math.toDegrees(fieldVel.omegaRadiansPerSecond);

      // Net turret angular rate in robot-relative frame
      double turretRateDegPerSec = trackingRateDegPerSec - robotOmegaDegPerSec;

      predictiveTurretAngleDeg = turretAngleDeg + turretRateDegPerSec * lookAheadSec;
      predictiveTurretAngleDeg = ((predictiveTurretAngleDeg % 360.0) + 360.0) % 360.0;

      // Keep predictive setpoint within physical limits.
      // If out of range, fall back to the instantaneous angle.
      if (predictiveTurretAngleDeg < config.turretMinAngleDeg
          || predictiveTurretAngleDeg > config.turretMaxAngleDeg) {
        predictiveTurretAngleDeg = turretAngleDeg;
      }
    }

    // -----------------------------------------------------------------------
    // Heading error for confidence (velocity-correction magnitude as proxy)
    // -----------------------------------------------------------------------
    double headingErrorDeg;
    if (velocityFiltered) {
      headingErrorDeg = 0;
    } else {
      double staticAngleDeg = toRobotRelative(
          Math.toDegrees(Math.atan2(ry, rx)), heading);
      double diff = turretAngleDeg - staticAngleDeg;
      diff = ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
      headingErrorDeg = Math.abs(diff);
    }

    // -----------------------------------------------------------------------
    // Solver convergence quality
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
        solverQuality = MathUtil.interpolate(1.0, 0.1, (double) (iterationsUsed - 3) / (maxIter - 3));
      }
    }

    double confidence = computeConfidence(
        solverQuality, robotSpeed, headingErrorDeg, distance, inputs.visionConfidence());

    previousSpeed = robotSpeed;

    return new LaunchParameters(
        effectiveRPMValue,
        effectiveTOFValue,
        turretAngleDeg,
        predictiveTurretAngleDeg,
        true,
        confidence,
        distance,
        iterationsUsed,
        warmStartUsed);
  }

  // -------------------------------------------------------------------------
  // Confidence scoring
  // -------------------------------------------------------------------------

  /**
   * 0-100 shot quality score. Weighted geometric mean of 5 factors.
   * Any single factor at zero collapses the whole score — intentional,
   * because you really shouldn't shoot if any one factor is missing.
   */
  private double computeConfidence(
      double solverQuality, double currentSpeed, double headingErrorDeg,
      double distance, double visionConfidence) {

    // 1. Solver convergence quality (0-1)
    double convergenceQuality = solverQuality;

    // 2. Velocity stability: penalise rapid speed changes
    double speedDelta = Math.abs(currentSpeed - previousSpeed);
    double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);

    // 3. Vision confidence (0-1, from caller)
    double visionConf = MathUtil.clamp(visionConfidence, 0, 1);

    // 4. Heading accuracy with speed + distance scaling.
    // Faster robot → tighter tolerance. Farther → tighter (same angle = larger
    // miss).
    double distanceScale = MathUtil.clamp(config.headingReferenceDistance / distance, 0.5, 2.0);
    double speedScale = 1.0 / (1.0 + config.headingSpeedScalar * currentSpeed);
    double scaledMaxError = config.turretMaxErrorDeg * distanceScale * speedScale;
    double headingAccuracy = MathUtil.clamp(1.0 - headingErrorDeg / scaledMaxError, 0, 1);

    // 5. Distance within scoring range: penalty near min/max boundaries
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

    double sumW = 0;
    double logSum = 0;
    for (int i = 0; i < 5; i++) {
      if (c[i] <= 0)
        return 0;
      logSum += w[i] * Math.log(c[i]);
      sumW += w[i];
    }

    return sumW <= 0 ? 0 : MathUtil.clamp(Math.exp(logSum / sumW) * 100.0, 0, 100);
  }

  // -------------------------------------------------------------------------
  // Correction / offset API
  // -------------------------------------------------------------------------

  /**
   * Layer a per-distance RPM adjustment on top of the base LUT. Good for field
   * tuning at comp.
   */
  public void addRpmCorrection(double distance, double deltaRpm) {
    correctionRpmMap.put(distance, deltaRpm);
  }

  /** Layer a per-distance TOF adjustment on top of the base LUT. */
  public void addTofCorrection(double distance, double deltaTof) {
    correctionTofMap.put(distance, deltaTof);
  }

  /** Clear all corrections, back to the raw LUT. */
  public void clearCorrections() {
    correctionRpmMap.clear();
    correctionTofMap.clear();
  }

  /** Bump the RPM offset by delta. Clamped to ±200 RPM. Bind to copilot D-pad. */
  public void adjustOffset(double delta) {
    rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
  }

  /**
   * Reset the RPM offset to zero. Call on mode transitions so trim doesn't carry
   * over.
   */
  public void resetOffset() {
    rpmOffset = 0;
  }

  public double getOffset() {
    return rpmOffset;
  }

  // -------------------------------------------------------------------------
  // Accessors
  // -------------------------------------------------------------------------

  /**
   * Raw time-of-flight from the LUT at this distance (no velocity compensation).
   */
  public double getTimeOfFlight(double distanceM) {
    return effectiveTOF(distanceM);
  }

  /** Base RPM at this distance, before any corrections or offset. */
  public double getBaseRPM(double distance) {
    if (shotLUT != null)
      return shotLUT.getRPM(distance);
    return rpmMap.get(distance);
  }

  /**
   * Reset warm-start state. Call after a pose reset so the solver doesn't
   * use stale data from the previous position.
   */
  public void resetWarmStart() {
    previousTOF = -1;
    previousSpeed = 0;
    prevRobotVx = 0;
    prevRobotVy = 0;
    prevRobotOmega = 0;
  }

  /**
   * Load a ShotLUT instead of calling loadLUTEntry() one at a time. RPM, hood
   * angle,
   * and TOF all interpolate together so they can't drift apart. Corrections from
   * addRpmCorrection() and copilot offset still layer on top.
   *
   * <p>
   * Takes priority over any entries added through loadLUTEntry().
   */
  public void loadShotLUT(ShotLUT lut) {
    this.shotLUT = lut;
  }

  /**
   * Hood angle at this distance from the ShotLUT.
   * Returns 0 if you loaded data through loadLUTEntry() (no angle in that path).
   */
  public double getHoodAngle(double distance) {
    if (shotLUT != null)
      return shotLUT.getAngle(distance);
    return 0;
  }

  // -------------------------------------------------------------------------
  // Package-private test helpers
  // -------------------------------------------------------------------------

  InterpolatingDoubleTreeMap getRpmMap() {
    return rpmMap;
  }

  InterpolatingDoubleTreeMap getTofMap() {
    return tofMap;
  }
}