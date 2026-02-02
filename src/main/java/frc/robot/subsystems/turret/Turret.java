package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private double lastPosition;
    private double deltaPosition;
    private int stillCounter;
    private enum homeHallEffectModes{
        START,
        MOVE_FORWARD,
        MOVE_REVERSE,
        DONE
    }
    private homeHallEffectModes homeHallEffectMode = homeHallEffectModes.START;
    private boolean homeDone = false;

    private enum botZones{
        ALLIANCE,
        NEUTRAL,
        OPPONENT
    }
    private Pose2d botPose;
    private botZones botZone = botZones.NEUTRAL;
    private double setPoint;

    private enum turretModes{
        HOMING,
        TRACKING_HUB,
        TRACKING_FEED,
        SHOOTING_HUB,
        SHOOTING_FEED
    }
    private turretModes turretMode = turretModes.HOMING;

    public Turret(TurretIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        switch (turretMode){
            case HOMING:
                homeWithHallEffect();
                break;
            case TRACKING_HUB:
                targetHub();
                break;
            case TRACKING_FEED:
                targetFeed();
                break;
            case SHOOTING_HUB:
                break;
            case SHOOTING_FEED:
                break;
        }
    }

    public void setTurretMode(turretModes mode){
        turretMode = mode;
    }

    public void homeWithHallEffect(){
        switch(homeHallEffectMode){
            case START:
                io.setIdleMode(IdleMode.kCoast);
                stillCounter = 0;
                lastPosition = inputs.positionRads;
                homeHallEffectMode = homeHallEffectModes.MOVE_FORWARD;
                break;
            case MOVE_FORWARD:
                io.setVoltage(2.0);
                deltaPosition = Math.abs(inputs.positionRads - lastPosition);
                lastPosition = inputs.positionRads;
                if(deltaPosition < 0.001){
                    stillCounter++;
                }
                else{
                    stillCounter = 0;
                }
                if(stillCounter > 50){
                        io.stop();
                        stillCounter = 0;
                        homeHallEffectMode = homeHallEffectModes.MOVE_REVERSE;
                }
                if(inputs.hallEffectTriggered){
                    io.stop();
                    io.setEncoder(0.0);
                    homeHallEffectMode = homeHallEffectModes.DONE;
                }
                break;
            case MOVE_REVERSE:
                io.setVoltage(-2.0);
                deltaPosition = Math.abs(inputs.positionRads - lastPosition);
                lastPosition = inputs.positionRads;
                if (deltaPosition < 0.001) {
                    stillCounter++;
                } else {
                    stillCounter = 0;
                }
                if(stillCounter > 50){
                    io.stop();
                    stillCounter = 0;
                    homeHallEffectMode = homeHallEffectModes.MOVE_FORWARD;
                }
                if(inputs.hallEffectTriggered){
                    io.stop();
                    io.setEncoder(0.0);
                    homeHallEffectMode = homeHallEffectModes.DONE;
                }
                break;
            case DONE:
                if(!homeDone){
                io.setIdleMode(IdleMode.kBrake);
                homeDone = true;
                }
                break;
        }
        
    }

    public void homeWithAbsEncoders(){
        double absEncoder1GR = (double)TurretConstants.turretGearboxTeeth / (double)TurretConstants.absEncoder1Teeth;
        double absEncoder2GR = (double)TurretConstants.turretGearboxTeeth / (double)TurretConstants.absEncoder2Teeth;

        double n = solveEquation(absEncoder1GR, absEncoder2GR, inputs.absPositionTours1, inputs.absPositionTours2);
        
        double turretTours = (n+inputs.absPositionTours1) * absEncoder1GR;
        double turretPosition = turretTours - Math.floor(turretTours);
        double positionRads = turretPosition * 2.0 * Math.PI;

        io.setEncoder(positionRads);
    }

    public void targetHub(){
        setPoint = setpointCalculation(TurretConstants.hubPose);
        io.setPosition(setPoint);
    }

    public void targetFeed(){
        switch(botZone){
            case ALLIANCE:
                break;
            case NEUTRAL:
                if(botPose.getY() < TurretConstants.zoneLine){
                    setPoint = setpointCalculation(TurretConstants.trenchPoseRight);
                    io.setPosition(setPoint);
                }
                else{
                    setPoint = setpointCalculation(TurretConstants.trenchPoseLeft);
                    io.setPosition(setPoint);
                }
                break;
            case OPPONENT:
                if(botPose.getY() < TurretConstants.zoneLine){
                    setPoint = setpointCalculation(TurretConstants.trenchPoseRight);
                    io.setPosition(setPoint);
                }
                else{
                    setPoint = setpointCalculation(TurretConstants.trenchPoseLeft);
                    io.setPosition(setPoint);
                }
                break;
        }
    }

    public Translation2d getBotPose(){
        if(turretMode == turretModes.TRACKING_HUB){
            return TurretConstants.hubPose;
        }
        else if(turretMode == turretModes.TRACKING_FEED){
            if(botZone == botZones.ALLIANCE){
                return null;
            }
            else if(botZone == botZones.NEUTRAL){
                if(botPose.getY() < TurretConstants.zoneLine){
                    return TurretConstants.trenchPoseRight;
                }
                else{
                    return TurretConstants.trenchPoseLeft;
                }
            }
            else{
                if(botPose.getY() < TurretConstants.zoneLine){
                    return TurretConstants.trenchPoseRight;
                }
                else{
                    return TurretConstants.trenchPoseLeft;
                }
            }
        }
        return null;
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

        double botAngleToTarget = Math.atan2(botToTargetY, botToTargetX);
        return botAngleToTarget - botPose.getRotation().getRadians();
    }

}
