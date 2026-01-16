// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.util.led.patterns.LedPattern;
import org.littletonrobotics.junction.Logger;

/**
 * Hardware-independent LED subsystem. Uses an AnimationController to manage
 * patterns and LedIO for hardware
 * abstraction.
 */
public class Led extends SubsystemBase {
    private final LedIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    private final AddressableLEDBuffer buffer;
    private final AnimationController controller;

    public Led() {
        System.out.println("[Led] Constructing Led Subsystem...");
        io = new LedIOReal(LedConstants.kLedPort, LedConstants.kLedLength);
        buffer = new AddressableLEDBuffer(LedConstants.kLedLength);
        controller = new AnimationController();

        // Default to black/off
    }

    @Override
    public void periodic() {
        // Update IO inputs
        io.updateInputs(inputs);

        // Update animation logic
        controller.update(buffer);

        // Update logging fields from controller state
        inputs.activePatternName = controller.getActivePatternName();
        Logger.processInputs("Led", inputs);

        // Send data to hardware/sim
        io.setData(buffer);

        // Log visual data for AdvantageScope (long array format)
        logLedData();
    }

    /**
     * Sets the entire strip to a static color.
     *
     * @param color The color to set.
     */
    /**
     * Sets a segment of the strip to a static color.
     *
     * @param color      The color to set.
     * @param startIndex Start index.
     * @param length     Length of the segment.
     */
    public void setStaticColor(Color color, int startIndex, int length) {
        LedPattern.SolidColor solid = new LedPattern.SolidColor(color);
        setAnimation(solid, startIndex, length);
    }

    /**
     * Sets the entire strip to a static color.
     *
     * @param color The color to set.
     */
    public void setStaticColor(Color color) {
        setStaticColor(color, 0, buffer.getLength());
    }

    /** Sets a segment of the strip to a static rgb value. */
    public void setStaticColor(int r, int g, int b, int startIndex, int length) {
        setStaticColor(new Color(r, g, b), startIndex, length);
    }

    /** Sets the entire strip to a static rgb value. */
    public void setStaticColor(int r, int g, int b) {
        setStaticColor(new Color(r, g, b));
    }

    /**
     * Starts a new animation on the LED strip.
     *
     * @param pattern The pattern to run.
     */
    public void setAnimation(LedPattern pattern) {
        if (pattern != null) {
            pattern.setLedCount(buffer.getLength());
            controller.setPattern(pattern, 0);
        } else {
            clearAnimation();
        }
    }

    /**
     * Starts a new animation on a segment of the LED strip.
     *
     * @param pattern    The pattern to run.
     * @param startIndex Start index for the pattern.
     * @param length     The number of LEDs in the segment.
     */
    public void setAnimation(LedPattern pattern, int startIndex, int length) {
        if (pattern != null) {
            pattern.setLedCount(length);
            controller.setPattern(pattern, startIndex);
        } else {
            clearAnimation();
        }
    }

    /**
     * Starts a new animation on a segment of the LED strip, using the remaining
     * length.
     *
     * @param pattern    The pattern to run.
     * @param startIndex Start index for the pattern.
     */
    public void setAnimation(LedPattern pattern, int startIndex) {
        if (pattern != null) {
            setAnimation(pattern, startIndex, buffer.getLength() - startIndex);
        } else {
            clearAnimation();
        }
    }

    /** Clears the current animation (sets to black/off). */
    public void clearAnimation() {
        controller.clear();
        setStaticColor(Color.kBlack);
    }

    /** Helper to log LED buffer to AdvantageScope format. */
    private void logLedData() {
        long[] data = new long[buffer.getLength()];
        for (int i = 0; i < buffer.getLength(); i++) {
            var color = buffer.getLED8Bit(i);
            data[i] = ((long) color.red << 16) | ((long) color.green << 8) | (long) color.blue;
        }
        Logger.recordOutput("Led/Data", data);
    }
}
