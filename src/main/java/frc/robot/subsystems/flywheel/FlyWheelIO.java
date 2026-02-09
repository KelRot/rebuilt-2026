package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean leadConnected;
    public double leadVelocityPerSec;
    public double leadAppliedVoltage;
    public double leadCurrentAmps;
    public double leadCurrentRpm;
    public double leadTargetRpm;
    public boolean followerConnected;
    public double followerVelocityPerSec;
    public double followerAppliedVoltage;
    public double followerCurrentAmps;
    public double followerCurrentRpm;
    public double followerTargetRpm;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void stop() {}

  default void setAppliedVoltage(double volts) {}

  default double getLeadVelocityRpm() {
    return 0.0;
  }

  default void setRpm(double targetRpm) {}
}
