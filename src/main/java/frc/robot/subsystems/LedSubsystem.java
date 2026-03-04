// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.hood.HoodIOSpark;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.util.led.Led;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.patterns.LedPattern;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
        private final Led led;
        private Superstructure superstructure = new Superstructure(
                        new Intake(new IntakeIO() {
                        }),
                        new Flywheel(new FlywheelIO() {
                        }),
                        new Kicker(new KickerIO() {
                        }),
                        new Hood(new HoodIOSpark() {
                        }),
                        new Turret(new TurretIO() {
                        }),
                        new Drive(
                                                        new GyroIOPigeon2(),
                                                        new ModuleIOSpark(0),
                                                        new ModuleIOSpark(1),
                                                        new ModuleIOSpark(2),
                                                        new ModuleIOSpark(3){
        
                                                        }),
        
                        new Index(new IndexIO() {
                        }));
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
                                break;
                        case SHOOTING_AND_INTAKING:
                                break;
                        case STUCKED_RECOVERY:
                                break;
                        default:
                                break;
                }
        }
}
