package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlyWheelIO {

  @AutoLog

  public static class FlywheelIOInputs {
    public boolean masterConnected;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double outputCurrentAmps;
    public double tempCelsius;  
    public double leadcurrentRpm;
    public double leadTargetRpm;

    public boolean followerConnected;
    public double followerSupplyCurrentAmps;
    public double followercurrentRpm;
    public double followerTargetRpm;
    public double followerTempCelsius;
  }
    default void updateInputs(FlywheelIOInputs inputs) {
    }

    public default void stopAllMotors() {
    }

    public default void setVoltage(double volts) {
    }
    
    public default double getLeadVelocity(double leadTargetRpm) {
          return leadTargetRpm;
    }
    public default void setRpm(double targetRpm) {
    }

    
}