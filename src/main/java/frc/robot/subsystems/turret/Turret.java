package frc.robot.subsystems.turret;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO io) {
        this.io = io;
    }

    public void hubAllign(){

    }

    public void feedAllign(){

    }

    public void homeTurret(){

    }

    public void setVoltage(double volts){
        io.setVoltage(volts);
    }

    public void setVelocity(double velocity){
        io.setVelocity(velocity);
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public void stop(){
        io.stop();
    }

    public void resetEncoder(){
        io.setEncoder(0);
    }
}