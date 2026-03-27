package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonTunablePID {

    private final TalonFX motor;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kS;

    public TalonTunablePID(
            TalonFX motor,
            String name,
            double p,
            double i,
            double d,
            double v,
            double s) {
        this.motor = motor;
        kP = new LoggedTunableNumber(name + "/kP", p);
        kI = new LoggedTunableNumber(name + "/kI", i);
        kD = new LoggedTunableNumber(name + "/kD", d);
        kV = new LoggedTunableNumber(name + "/kV", v);
        kS = new LoggedTunableNumber(name + "/kS", s);
    }

    // kV ve kS olmadan kullanmak isteyenler için
    public TalonTunablePID(
            TalonFX motor,
            String name,
            double p,
            double i,
            double d) {
        this(motor, name, p, i, d, 0.0, 0.0);
    }

    public void periodic() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                values -> {
                    Slot0Configs slot0 = new Slot0Configs();
                    slot0.kP = values[0];
                    slot0.kI = values[1];
                    slot0.kD = values[2];
                    slot0.kV = values[3];
                    slot0.kS = values[4];
                    motor.getConfigurator().apply(slot0);
                },
                kP, kI, kD, kV, kS);
    }

    public double getP() { return kP.get(); }
    public double getI() { return kI.get(); }
    public double getD() { return kD.get(); }
    public double getV() { return kV.get(); }
    public double getS() { return kS.get(); }
}