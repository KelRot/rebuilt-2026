package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.Led;
import frc.robot.util.led.patterns.LedPattern;

public class LedSubsystem extends SubsystemBase {
    private final Led led;
    private final Superstructure superstructure;
    private Superstructure.SuperstructureState lastState = null;

    public LedSubsystem(Led led, Superstructure superstructure) {
        this.led = led;
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        Superstructure.SuperstructureState currentState = superstructure.getCurrentState();

        if (currentState == lastState) return;
        lastState = currentState;

        switch (currentState) {
            case OPENING_INTAKE:
                led.setAnimation(new LedPattern.Fire(1.0, 55, 120, led.getLength()));
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
                led.setAnimation(new LedPattern.Larson(Color.kRed, 1, 4.0));
                break;
            case PREP_SHOOTING_AND_INTAKING:
                led.setAnimation(new LedPattern.Larson(Color.kOrange, 1, 4.0));
                break;
            case SHOOTING:
                led.setAnimation(new LedPattern.ColorFlow(Color.kRed, LedPattern.ColorFlow.Direction.FORWARD, 1.0));
                break;
            case SHOOTING_AND_INTAKING:
                led.setAnimation(new LedPattern.ColorFlow(Color.kOrangeRed, LedPattern.ColorFlow.Direction.FORWARD, 1.0));
                break;
            case STUCKED_RECOVERY:
                led.setAnimation(new LedPattern.Strobe(Color.kPurple, 0.15));
                break;
            case DEFAULT:
                led.setAnimation(new LedPattern.Breathe(Color.kPowderBlue, 1.2));
                break;
            case TESTING:
                led.setAnimation(new LedPattern.Strobe(Color.kWhite, 0.25));
                break;
            case IDLE:
            default:
                led.setAnimation(new LedPattern.Rainbow(1.5));
                break;
        }
    }
}