// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedIOReal implements LedIO {
    private final AddressableLED m_led;

    public LedIOReal(int port, int length) {
        m_led = new AddressableLED(port);
        m_led.setLength(length);
        m_led.start();
        System.out.println("[LedIOReal] Initialized on PWM Port " + port + " with Length " + length
                + ", Verify this matches your wiring!");
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.connected = true;
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        m_led.setData(buffer);
    }
}
