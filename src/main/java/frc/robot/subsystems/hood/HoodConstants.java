package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class HoodConstants {
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    
    public static final int hoodID = 0;
    public static final int absEncoder1ID = 0;
    public static final int absEncoder2ID = 0;
    public static final int hallEffectID = 0;

    public static final double positionConversionFactor = 0; // Example conversion factor
    public static final double velocityConversionFactor = 0; 

    public static final DCMotor hoodGearbox = DCMotor.getNEO(0);
    public static final double hoodMotorReduction = 0;

    public static final double absolutePositionOffsetRads = 0.0;

    public static final double minAngle = -270;
    public static final double maxAngle = 270;

    public static final double cruiseVelocity = 0.0;
    public static final double maxAcceleration = 0.0;

    public static final Translation2d hubPose = new Translation2d(0.0, 0.0);
    public static final double hubHeight = 10; 
    public static final double shooterHeight = 10; 
    public static final Translation2d trenchPoseRight = new Translation2d(0.0, 0.0);
    public static final Translation2d trenchPoseLeft = new Translation2d(0.0, 0.0);
    public static final double zoneLine = 0.0;

    public static final int hoodGearboxTeeth = 0;
    public static final int absEncoder1Teeth = 0;
    public static final int absEncoder2Teeth = 0;
    
}
