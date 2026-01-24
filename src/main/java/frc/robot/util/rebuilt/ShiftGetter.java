package frc.robot.util.rebuilt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;

public class ShiftGetter {
    private static boolean check;
    private static boolean isReal;

    public ShiftGetter() {
    }

    public enum FirstInactiveShift { // Auto winner, i guess.
        BLUE,
        RED,
        NONE
    }

    public enum Shift {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        NONE
    }

    public static FirstInactiveShift getFirstInactiveShift() {
        String data = DriverStation.getGameSpecificMessage();

        FirstInactiveShift result;
        if (data == null || data.isEmpty()) {
            result = FirstInactiveShift.NONE;
        } else {
            result = switch (data.charAt(0)) {
                case 'B' -> FirstInactiveShift.BLUE;
                case 'R' -> FirstInactiveShift.RED;
                default -> FirstInactiveShift.NONE;
            };
        }

        Logger.recordOutput("/Shift/FirstInactive", result.name());
        return result;
    }

    public static boolean isFirstInactiveOwnAllianceShift() {
        boolean ownInactive = DriverStation.getAlliance()
                .map(alliance -> (alliance == DriverStation.Alliance.Blue
                        && getFirstInactiveShift() == FirstInactiveShift.BLUE)
                        || (alliance == DriverStation.Alliance.Red
                                && getFirstInactiveShift() == FirstInactiveShift.RED))
                .orElse(false);

        Logger.recordOutput("/Shift/OwnFirstInactive", ownInactive);
        return ownInactive;
    }

    private static boolean isRealMatch() {
        if (!DriverStation.isEnabled()) {
            check = false;
            isReal = false;

            Logger.recordOutput("/Shift/IsRealMatch", false);
            return false;
        }

        if (!check) {
            isReal = DriverStation.isFMSAttached()
                    || DriverStation.getMatchTime() > 15.0;
            check = true;
        }

        Logger.recordOutput("/Shift/IsRealMatch", isReal);
        return isReal;
    }

    public static boolean isActive() {
        if (!isRealMatch()) {
            Logger.recordOutput("/Shift/IsActive", true);
            return true;
        }

        Shift shift = getCurrentShift();

        // Her zaman aktif olanlar
        if (shift == Shift.AUTO || shift == Shift.TRANSITION || shift == Shift.ENDGAME) {
            Logger.recordOutput("/Shift/IsActive", true);
            return true;
        }

        boolean ownInactive = isFirstInactiveOwnAllianceShift();

        boolean active = ownInactive
                ? (shift == Shift.SHIFT_2 || shift == Shift.SHIFT_4)
                : (shift == Shift.SHIFT_1 || shift == Shift.SHIFT_3);

        Logger.recordOutput("/Shift/IsActive", active);
        return active;
    }

    public static Shift getCurrentShift() {
        double t = DriverStation.getMatchTime();
        Logger.recordOutput("/Shift/MatchTime", t);

        Shift shift;
        if (t < 0) {
            shift = Shift.NONE;
        } else if (DriverStation.isAutonomous()) {
            shift = Shift.AUTO;
        } else if (t > 130) {
            shift = Shift.TRANSITION;
        } else if (t > 105) {
            shift = Shift.SHIFT_1;
        } else if (t > 80) {
            shift = Shift.SHIFT_2;
        } else if (t > 55) {
            shift = Shift.SHIFT_3;
        } else if (t > 30) {
            shift = Shift.SHIFT_4;
        } else {
            shift = Shift.ENDGAME;
        }

        Logger.recordOutput("/Shift/Current", shift.name());
        return shift;
    }
}
