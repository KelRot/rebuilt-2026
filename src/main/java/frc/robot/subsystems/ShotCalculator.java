package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.util.rebuilt.field.Field;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TurretConstants.robotToTurret;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.Constants.TurretConstants;



public class ShotCalculator {
    //Distance from the target to the center of the robot
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }


    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ()
                .minus(robotToTurret.getMeasureZ())
                .in(Inches);
        double angle = Math.atan(
                ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
                        / (g * x_dist));
        return Radians.of(angle);
    }

    // calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = Math.PI / 2 - hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    public static ShotData calculateShotFromFunnelClearance(Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double y_dist = predictedTarget.getMeasureZ().minus(robotToTurret.getMeasureZ()).in(Inches);
        double g = 386;
        double r = Field.BlueHub.innerOpeningWidth/2* x_dist/getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = Field.BlueHub.innerOpeningHeight+2;
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
        if (Double.isNaN(v0) || Double.isNaN(theta)) {
            v0 = 0;
            theta = 0;
        }
        return new ShotData(
                linearToAngularVelocity(InchesPerSecond.of(v0), FlywheelConstants.flywheelRadius),
                Radians.of(Math.PI / 2 - theta),
                predictedTarget);
    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShotFromFunnelClearance(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }

    public static ShotData iterativeMovingShotFromMap(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        double distance = getDistanceToTarget(robot, target).in(Meters);
        ShotData shot = SHOT_MAP.get(distance);
        shot = new ShotData(shot.exitVelocity, shot.hoodAngle, target);
        Time timeOfFlight = Seconds.of(TOF_MAP.get(distance));
        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getDistanceToTarget(robot, predictedTarget).in(Meters);
            shot = SHOT_MAP.get(distance);
            shot = new ShotData(shot.exitVelocity, shot.hoodAngle, predictedTarget);
            timeOfFlight = Seconds.of(TOF_MAP.get(distance));
        }

        return shot;
    }


    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target){
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
        }

        public LinearVelocity getExitVelocity() {
            return angularToLinearVelocity(RadiansPerSecond.of(this.exitVelocity), FLYWHEEL_RADIUS);
        }

        public Angle getHoodAngle() {
            return Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }

        public static ShotData interpolate(ShotData start, ShotData end, double t) {
            return new ShotData(
                    MathUtil.interpolate(start.exitVelocity, end.exitVelocity, t),
                    MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
                    end.target);
        }
    }


    
}
