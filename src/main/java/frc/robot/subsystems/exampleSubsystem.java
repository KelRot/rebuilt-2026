package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motors.MotorIO.Motor;
import frc.robot.util.motors.controllers.SparkBaseController;

public class exampleSubsystem extends SubsystemBase {

    private final SparkBaseController motor;

    public exampleSubsystem(Motor motor) {

        this.motor = new SparkBaseController(motor);
        
    }

}
