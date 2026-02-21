package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBaseExtensions;

public final class BrakeController {

    private static final List<SparkBase> motors = new ArrayList<>();

    private BrakeController() {}

    public static void register(SparkBase motor) {
        motors.add(motor);
    }

    public static void setAllBrake(boolean brake) {
        for (SparkBase motor : motors) {
            SparkBaseExtensions.setBrakeMode(motor, brake);
        }
    }
}