package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.wantedState;

import java.util.function.DoubleSupplier;

public class ShootCommands extends Command {

    private final Flywheel flywheel;
    private final DoubleSupplier distanceSupplier;

    /**
     * @param flywheel Flywheel subsystem
     * @param distanceSupplier Mesafeyi metre cinsinden veren supplier
     *                         (vision, pose, vs.)
     */
    public ShootCommands(Flywheel flywheel, DoubleSupplier distanceSupplier) {
        this.flywheel = flywheel;
        this.distanceSupplier = distanceSupplier;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setGoal(wantedState.SHOOTING);
    }

    @Override
    public void execute() {
        double distance = distanceSupplier.getAsDouble();
        flywheel.setTargetDistance(distance);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setGoal(wantedState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
