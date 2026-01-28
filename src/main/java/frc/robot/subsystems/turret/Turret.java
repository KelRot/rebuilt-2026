package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private double lastPosition;
    private double deltaPosition;
    private int stillCounter;
    private enum homeModes{
        START,
        MOVE_FORWARD,
        MOVE_REVERSE,
        DONE
    }
    private homeModes homeMode = homeModes.START;
    private boolean homeDone = false;

    private enum botZones{
        ALLIANCE,
        NEUTRAL,
        OPPONENT
    }
    private Pose2d botPose;
    private Translation2d hubPose = TurretConstants.hubPose;
    private Translation2d trenchPoseUpper = TurretConstants.trenchPoseUpper;
    private Translation2d trenchPoseLower = TurretConstants.trenchPoseLower;
    private double botToHubX;
    private double botToHubY;
    private double botToTrenchX;
    private double botToTrenchY;
    private botZones botZone = botZones.NEUTRAL;

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
                home();
                break;
            case TRACKING_HUB:
                break;
            case TRACKING_FEED:
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

    public void home(){
        switch(homeMode){
            case START:
                io.setIdleMode(IdleMode.kCoast);
                stillCounter = 0;
                lastPosition = inputs.positionRads;
                homeMode = homeModes.MOVE_FORWARD;
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
                        homeMode = homeModes.MOVE_REVERSE;
                }
                if(inputs.hallEffectTriggered){
                    io.stop();
                    io.zeroEncoder();
                    homeMode = homeModes.DONE;
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
                    homeMode = homeModes.MOVE_FORWARD;
                }
                if(inputs.hallEffectTriggered){
                    io.stop();
                    io.zeroEncoder();
                    homeMode = homeModes.DONE;
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

    public void targetHub(){
        botToHubX = hubPose.getX() - botPose.getX();
        botToHubY = hubPose.getY() - botPose.getY();

        double botAngleToHub = Math.atan2(botToHubY, botToHubX);
        double setPoint = botAngleToHub - hubPose.getZ();
        io.setPosition(setPoint);
    }

    public void targetFeed(){
        switch(botZone){
            case ALLIANCE:

                break;
            case NEUTRAL:

                break;
            case OPPONENT:

                break;
        }
    }

}
