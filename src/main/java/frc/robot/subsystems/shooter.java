package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    public static SparkFlex vortex1, vortex2;
    public static SparkFlexConfig   vortexConfig;
  /** Creates a new ExampleSubsystem. */
  public shooter() {
    vortex1 = new SparkFlex(25, SparkFlex.MotorType.kBrushless);
    vortex2 = new SparkFlex(26, SparkFlex.MotorType.kBrushless);
    vortexConfig = new SparkFlexConfig();
    vortexConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(60);
    vortex1.configure(vortexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    vortexConfig.follow(vortex1).inverted(true);
    vortex2.configure(vortexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    SmartDashboard.putNumber("Voltage", 0);
  }


  @Override
  public void periodic() {
    double num = SmartDashboard.getNumber("Voltage", 0);
    vortex1.setVoltage(num);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
