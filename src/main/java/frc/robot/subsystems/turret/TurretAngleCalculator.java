package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TurretConstants;

public class TurretAngleCalculator {

    private static final int SMALL_GEAR = 24;
    private static final int BIG_GEAR = 25;
    private static final int DRIVEN_GEAR = 108;
    private static final int SCAN_RANGE = 10;

    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(540);

    private static final double SMALL_ENCODER_OFFSET = 0.703602917590073;
    private static final double BIG_ENCODER_OFFSET = 0.7689608942240224;

    private final DutyCycleEncoder smallEncoder;
    private final DutyCycleEncoder bigEncoder;

    private Rotation2d lastCalculatedAngle = Rotation2d.kZero;
    private Rotation2d lastCRTError = Rotation2d.kZero;
    private boolean isCalibrated = false;

    public TurretAngleCalculator() {
        smallEncoder = new DutyCycleEncoder(TurretConstants.absEncoder1ID);
        bigEncoder = new DutyCycleEncoder(TurretConstants.absEncoder2ID);
        SmartDashboard.putData("Calibrate Command", Commands.runOnce(() -> calibrate()));
    }

    public void log() {
        Rotation2d angle = calculateTurretAngle();
        if (angle != null) {
            lastCalculatedAngle = angle;
            isCalibrated = true;
        }

        SmartDashboard.putNumber("Turret/CRT/AngleDeg", lastCalculatedAngle.getDegrees());
        SmartDashboard.putNumber("Turret/CRT/ErrorDeg", lastCRTError.getDegrees());
        SmartDashboard.putNumber(
                "Turret/CRT/SmallEncoderRaw",
                smallEncoder.isConnected() ? applyOffset(smallEncoder.get(), SMALL_ENCODER_OFFSET) : -1);
        SmartDashboard.putNumber(
                "Turret/CRT/BigEncoderRaw",
                bigEncoder.isConnected() ? applyOffset(bigEncoder.get(), BIG_ENCODER_OFFSET) : -1);
        SmartDashboard.putBoolean("Turret/CRT/SmallEncOK", smallEncoder.isConnected());
        SmartDashboard.putBoolean("Turret/CRT/BigEncOK", bigEncoder.isConnected());
        SmartDashboard.putBoolean("Turret/CRT/IsCalibrated", isCalibrated);
        SmartDashboard.putBoolean("Turret/CRT/CRTHealthy", lastCRTError.getDegrees() < 3.0);
    }

    private Rotation2d calculateTurretAngle() {
        if (!smallEncoder.isConnected() || !bigEncoder.isConnected())
            return null;

        double smallPos = applyOffset(smallEncoder.get(), SMALL_ENCODER_OFFSET);
        double bigPos = applyOffset(bigEncoder.get(), BIG_ENCODER_OFFSET);

        double[] smallPositions = new double[SCAN_RANGE];
        double[] bigPositions = new double[SCAN_RANGE];

        for (int i = 0; i < SCAN_RANGE; i++) {
            smallPositions[i] = (i + smallPos) * SMALL_GEAR / DRIVEN_GEAR;
            bigPositions[i] = (i + bigPos) * BIG_GEAR / DRIVEN_GEAR;
        }

        double out = 0;
        double minDiff = Double.MAX_VALUE;

        for (int i = 0; i < SCAN_RANGE; i++) {
            for (int z = 0; z < SCAN_RANGE; z++) {
                double diff = Math.abs(smallPositions[i] - bigPositions[z]);
                if (diff < minDiff) {
                    out = (smallPositions[i] + bigPositions[z]) / 2.0;
                    minDiff = diff;
                }
            }
        }

        lastCRTError = Rotation2d.fromRotations(minDiff);
        return Rotation2d.fromRotations(out);
    }

    public Rotation2d getAngle() {
        return lastCalculatedAngle;
    }

    public Rotation2d getCRTError() {
        return lastCRTError;
    }

    public boolean isHealthy() {
        return smallEncoder.isConnected()
                && bigEncoder.isConnected()
                && lastCRTError.getDegrees() < 3.0;
    }

    public boolean isSafeAngle(Rotation2d angle) {
        return angle.getDegrees() >= MIN_ANGLE.getDegrees()
                && angle.getDegrees() <= MAX_ANGLE.getDegrees();
    }

    public void calibrate() {
        if (!smallEncoder.isConnected() || !bigEncoder.isConnected()) {
            System.err.println("One of the encoders is not connected! Cannot calibrate.");
            return;
        }
        double s = smallEncoder.get();
        double b = bigEncoder.get();
        System.out.println("  SMALL_ENCODER_OFFSET = " + s);
        System.out.println("  BIG_ENCODER_OFFSET   = " + b);

        SmartDashboard.putNumber("Turret/CRT/CalibSmall", s);
        SmartDashboard.putNumber("Turret/CRT/CalibBig", b);
    }

    private double applyOffset(double raw, double offset) {
        return ((raw - offset) % 1.0 + 1.0) % 1.0;
    }


    public double getBigEncoder() {
        return bigEncoder.get();
    }
    public double getSmallEncoder() {
        return smallEncoder.get();
    }
}