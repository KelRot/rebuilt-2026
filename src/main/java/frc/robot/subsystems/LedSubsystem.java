// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.led.Led;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.patterns.LedPattern;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
    private final Led led;
    private Superstructure superstructure;
    private SuperstructureState currentState = null; // ✅ eksik değişken eklendi

    public LedSubsystem(Led led, Superstructure superstructure) {
        this.led = led;
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        SuperstructureState newState = superstructure.getCurrentState(); // ✅ wantedState yerine direkt al
        if (newState != currentState) {
            currentState = newState;
            applyState(); // ✅ applyState() çağrısı düzeltildi
        }
    }

    private void applyState() { // ✅ parametre kaldırıldı (zaten kullanılmıyordu)
        switch (superstructure.getCurrentState()) {
            case OPENING_INTAKE:
                led.setAnimation(new LedPattern.Fire(1.0, 55, 120, LedConstants.kLedLength));
                break;
            case INTAKING:
                led.setAnimation(new LedPattern.Breathe(Color.kLime, 1.2));
                break;
            case CLOSING_AND_STOPPING_INTAKE:
                led.setAnimation(new LedPattern.SingleFade(Color.kForestGreen, 2));
                break;
            case REJECTING_INTAKE:
                led.setAnimation(new LedPattern.TwinkleOff(Color.kGold, 1.2, 8));
                break;
            case PREP_SHOOTING:
                led.setAnimation(new LedPattern.Larson(Color.kMediumAquamarine, 1, 4.0));
                break;
            case SHOOTING:
                led.setAnimation(new LedPattern.ColorFlow(Color.kPeru, null, 1.0));
                break;
            case DEFAULT:
                led.setAnimation(new LedPattern.Breathe(Color.kPowderBlue, 1.2));
                break;
            case TESTING:
                led.setAnimation(new LedPattern.Strobe(Color.kWhite, 0.25));
                break;
            case IDLE:
                led.setAnimation(new LedPattern.Rainbow(1.5));
                break;
            case PREP_SHOOTING_AND_INTAKING:
            case SHOOTING_AND_INTAKING:
            case STUCKED_RECOVERY:
                break;
            default:
                led.setAnimation(new LedPattern.Rainbow(1.5));
                break;
        }
        // ✅ sondaki led.setAnimation(Rainbow) KALDIRILDI — switch'i eziyordu!
    }
}