package frc.robot.subsystems.hood;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private double lastPosition;
    private double deltaPosition;
    private int stillCounter;
    private enum homeHallEffectModes{
        START,
        MOVE_REVERSE,
        DONE
    }
    private homeHallEffectModes homeHallEffectMode = homeHallEffectModes.START;
    private boolean homeDone = false;

    private Pose2d botPose;
    private double setPoint;

    private enum hoodModes{
        HOMING,
        TRACKING_HUB,
        TRACKING_FEED,
    }
    private hoodModes hoodMode = hoodModes.HOMING;

    public Hood(HoodIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        switch (hoodMode){
            case HOMING:
                homeWithHallEffect();
                break;
            case TRACKING_HUB:
                targetHub();
                break;
            case TRACKING_FEED:
                targetFeed();
                break;
        }
    }

    public void setHoodMode(hoodModes mode){
        hoodMode = mode;
    }

    public void homeWithHallEffect(){
        switch(homeHallEffectMode){
            case START:
                io.setIdleMode(IdleMode.kCoast);
                stillCounter = 0;
                lastPosition = inputs.positionRads;
                homeHallEffectMode = homeHallEffectModes.MOVE_REVERSE;
                break;
            case MOVE_REVERSE:
                //io.setVoltage(-2.0);
                //deltaPosition = Math.abs(inputs.positionRads - lastPosition);
                //lastPosition = inputs.positionRads;
                //if (deltaPosition < 0.001) {
                //    stillCounter++;
                //} else {
                //    stillCounter = 0;
                //}
                //if(stillCounter > 50){
                //    io.stop();
                //    stillCounter = 0;
                //    homeHallEffectMode = homeHallEffectModes.MOVE_FORWARD;
                //}
                //if(inputs.hallEffectTriggered){
                //    io.stop();
                //    io.setEncoder(0.0);
                //    homeHallEffectMode = homeHallEffectModes.DONE;
                //}
                //break;
            case DONE:
                if(!homeDone){
                io.setIdleMode(IdleMode.kBrake);
                homeDone = true;
                }
                break;
        }
        
    }

    public void targetHub(){
        setPoint = setpointCalculation(HoodConstants.hubPose);
        io.setPosition(setPoint);
    }

    public void targetFeed(){
        //switch(botZone){
        //    case ALLIANCE:
        //        break;
        //    case NEUTRAL:
        //        if(botPose.getY() < TurretConstants.zoneLine){
        //            setPoint = setpointCalculation(TurretConstants.trenchPoseRight);
        //            io.setPosition(setPoint);
        //        }
        //        else{
        //            setPoint = setpointCalculation(TurretConstants.trenchPoseLeft);
        //            io.setPosition(setPoint);
        //        }
        //        break;
        //    case OPPONENT:
        //        if(botPose.getY() < TurretConstants.zoneLine){
        //            setPoint = setpointCalculation(TurretConstants.trenchPoseRight);
        //            io.setPosition(setPoint);
        //        }
        //        else{
        //            setPoint = setpointCalculation(TurretConstants.trenchPoseLeft);
        //            io.setPosition(setPoint);
        //        }
        //        break;
        //}
    }

    public Translation2d getBotPose(){
        //if(hoodMode == hoodModes.TRACKING_HUB){
        //    return HoodConstants.hubPose;
        //}
        //else if(hoodMode == hoodModes.TRACKING_FEED){
        //    if(botZone == botZones.ALLIANCE){
        //        return null;
        //    }
        //    else if(botZone == botZones.NEUTRAL){
        //        if(botPose.getY() < TurretConstants.zoneLine){
        //            return TurretConstants.trenchPoseRight;
        //        }
        //        else{
        //            return TurretConstants.trenchPoseLeft;
        //        }
        //    }
        //    else{
        //        if(botPose.getY() < TurretConstants.zoneLine){
        //            return TurretConstants.trenchPoseRight;
        //        }
        //        else{
        //            return TurretConstants.trenchPoseLeft;
        //        }
        //    }
        //}
        //return null;
    }

    public double solveEquation(double gr1, double gr2, double absPos1, double absPos2){
        double a = gr1;
        double b = absPos1;
        double c = gr2;
        double d = absPos2;

        double n = ((c*d)-(b*a))/(a - c);
        return n;
    }

    public double setpointCalculation(Translation2d targetPose){
         double botToTargetX = targetPose.getX() - botPose.getX();
    double botToTargetY = targetPose.getY() - botPose.getY();

    double distance = Math.hypot(botToTargetX, botToTargetY);

    double hoodAngleToTarget = Math.atan2(
        HoodConstants.hubHeight - HoodConstants.shooterHeight,
        distance
    );

    return hoodAngleToTarget;
    }
}