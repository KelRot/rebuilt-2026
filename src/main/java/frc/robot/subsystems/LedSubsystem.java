// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.Led;
import frc.robot.util.led.patterns.LedPattern;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
        private final Led led;
        private final Superstructure superstructure;
        public LedSubsystem(Led led, Superstructure superstructure) {
                System.out.println("[LedSubsystem] Constructing LedSubsystem...");
                this.led = led;
                this.superstructure = superstructure;
                led.setStaticColor(Color.kAliceBlue);
        }

        @Override
        public void periodic() {        
                switch(superstructure.getCurrentState()) {
                        case OPENING_INTAKE:
                                led.setAnimation(new LedPattern.Fire(1.0,55,120,LedConstants.kLedLength));
                                break;
                        case INTAKING:
                                led.setAnimation(new LedPattern.Breathe(Color.kLime, 1.2));
                                break;
                        case CLOSING_INTAKE:
                                led.setAnimation(new LedPattern.SingleFade(Color.kForestGreen, 2));
                                break;
                        case REJECTING_INTAKE:
                                led.setAnimation(new LedPattern.TwinkleOff(Color.kGold, 1.2, 8));
                                break;
                        case OUTTAKE:
                                led.setAnimation(new LedPattern.Strobe(Color.kDodgerBlue, 0.25));
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
                        case STOP:
                                led.setAnimation(new LedPattern.Rainbow(1.5));
                                break;
                        case IDLE:
                                led.setAnimation(new LedPattern.Rainbow(1.5));
                                break;
                }
        }
}
