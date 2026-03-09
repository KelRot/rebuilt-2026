package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkTunablePID {

    public enum DriveType {
        VORTEX,
        MAX
    }

    private final SparkBase motor;
    private SparkBaseConfig config;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;

    public SparkTunablePID(
            SparkBase motor,
            String name,
            DriveType type,
            double p,
            double i,
            double d) {

        this.motor = motor;
        switch (type) {
            case MAX:
                config = new SparkMaxConfig();
            case VORTEX:
                config = new SparkFlexConfig();
            default:
                config = new SparkMaxConfig();
        }
        kP = new LoggedTunableNumber(name + "/kP", p);
        kI = new LoggedTunableNumber(name + "/kI", i);
        kD = new LoggedTunableNumber(name + "/kD", d);
    }

    public void periodic() {

        LoggedTunableNumber.ifChanged(
                hashCode(),
                values -> {
                    config.closedLoop.pid(values[0], values[1], values[2]);

                    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

                },
                kP, kI, kD);
    }

    public double getP() {
        return kP.get();
    }

    public double getI() {
        return kI.get();
    }

    public double getD() {
        return kD.get();
    }
}
