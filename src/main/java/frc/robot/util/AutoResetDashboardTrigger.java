package frc.robot.util;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
* AutoResetDashboardTrigger
* -------------------------
* A dashboard-backed one-shot trigger designed to behave like
* a gamepad "onTrue" button.
*
* Behavior:
* - Exposes a Shuffleboard Toggle Button
* - Detects a FALSE -> TRUE transition (rising edge)
* - Fires TRUE for exactly one scheduler loop
* - Automatically resets the dashboard value back to FALSE
*
* Intended use cases:
* - Single-action commands (reset, zeroing, shoot-once, test actions)
* - Debug and pit utilities where holding a button is unsafe
*
* Not intended for:
* - whileTrue / continuous commands
* - toggle or latch-style behaviors
*/
public final class AutoResetDashboardTrigger {


/** NetworkTables-backed boolean entry shown on Shuffleboard */
private final GenericEntry buttonEntry;


/** Cached previous value for edge detection */
private boolean previousValue = false;


/**
* Creates an auto-resetting dashboard trigger.
*
* @param tabName Shuffleboard tab name
* @param buttonName Label displayed on the dashboard
*/
public AutoResetDashboardTrigger(String tabName, String buttonName) {
ShuffleboardTab tab = Shuffleboard.getTab(tabName);


this.buttonEntry = tab.add(buttonName, false)
.withWidget("Toggle Button")
.getEntry();
}


/**
* Returns a WPILib Trigger that fires exactly once
* when the dashboard button is pressed.
*
* @return one-shot Trigger (rising-edge based)
*/
public Trigger trigger() {
return new Trigger(() -> {
boolean currentValue = buttonEntry.getBoolean(false);


// Rising edge detection
if (currentValue && !previousValue) {
buttonEntry.setBoolean(false); // auto-reset immediately
previousValue = true;
return true;
}


previousValue = currentValue;
return false;
});
}


/**
* Programmatically fires the trigger once.
* Useful for tests or internal control logic.
*/
public void fireOnce() {
buttonEntry.setBoolean(true);
}
}


/*
========================
Example usage (RobotContainer)
========================


AutoResetDashboardTrigger shootOnce =
new AutoResetDashboardTrigger("Driver", "Shoot Once");


shootOnce.trigger()
.onTrue(new ShootCommand());


// Designed for onTrue only
// whileTrue / toggle usage is intentionally unsupported
*/