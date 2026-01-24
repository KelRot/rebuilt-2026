package frc.robot.util.rebuilt.field;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drive.RobotZone;

public class Zones {

    private static final Drive drive = RobotContainer.getDrive();

    public static final Trigger blueFieldSide = new Trigger(() -> drive.getRobotZone() == RobotZone.BLUE_ALLIANCE_ZONE);
    public static final Trigger opponentFieldSide = new Trigger(() -> blueFieldSide.getAsBoolean() != Field.isBlue() && drive.getRobotZone() != RobotZone.NEUTRAL_ZONE);
    public static final Trigger neutralFieldSide = new Trigger(() -> drive.getRobotZone() == RobotZone.NEUTRAL_ZONE);

}