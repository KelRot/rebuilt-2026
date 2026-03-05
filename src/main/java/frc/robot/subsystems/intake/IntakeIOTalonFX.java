package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;

import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO{
    private static TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.rollerMotorID);

    public IntakeIOTalonFX(){
        config();
    }
    
    @Override
    public void setRollerVoltage(double voltage){
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerConnected = ;
        inputs.rollerMotorCurrentAmps = rollerMotor.getStatorCurrent().getValue();
        inputs.rollerMotorVoltageVolts = rollerMotor.getMotorVoltage().getValue();
        inputs.rollerMotorSpeedRpm = rollerMotor.getVelocity().getValue() * 60;
    }

    public void config() {
        rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    
        rollerMotor.getConfigurator().apply(rollerConfig);
    }
}
