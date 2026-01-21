package com.revrobotics.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;

public class SparkBaseExtensions {

    /**
     * Checked function for setting controller inverted.
     *
     * <p>
     * This call has no effect if the controller is a follower.
     *
     * @param isInverted true = inverted
     */
    public static REVLibError setInverted(SparkBase sparkBase, boolean isInverted) {
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetInverted(sparkBase.sparkHandle, isInverted));
    }

    /**
     * Enable or disable center aligned mode for the duty cycle sensor.
     *
     * @param enable true = center aligned enabled
     */
    public static REVLibError setCenterAlignedMode(SparkBase sparkBase, boolean enable) {
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetParameterBool(sparkBase.sparkHandle, 152, enable));
    }

    /**
     * Enable or disable PID voltage output mode.
     *
     * <p>
     * When enabled, PID output is voltage instead of duty cycle.
     *
     * @param enable true = PID voltage output
     */
    public static REVLibError setPIDVoltageOutput(SparkBase sparkBase, boolean enable) {
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetParameterUint32(
                        sparkBase.sparkHandle, 74, enable ? 1 : 0));
    }

    /**
     * Enable or disable brake mode.
     *
     * <p>
     * true = Brake mode
     * false = Coast mode
     *
     * @param brake true = brake, false = coast
     */
    public static REVLibError setBrakeMode(SparkBase sparkBase, boolean brake) {
        // 1 = Brake, 0 = Coast
        return REVLibError.fromInt(
                CANSparkJNI.c_Spark_SetParameterUint32(
                        sparkBase.sparkHandle,
                        6,
                        brake ? 1 : 0));
    }
}
