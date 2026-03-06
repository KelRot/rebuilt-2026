// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSparkFlex;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.index.IndexIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.Intake.SystemState;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSpark;
import frc.robot.util.FuelSim;
import frc.robot.util.led.Led;
import lombok.Getter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * button mappings) should be declared here.
 */
public class RobotContainer {

        // Subsystems
        @Getter
        public static Kicker kicker;
        @Getter
        public static Led led;
        @Getter
        public static Intake intake;
        @Getter
        public static Index index;
        @Getter
        public static Flywheel flywheel;
        @Getter
        public static Superstructure superstructure;
        // Controller
        @Getter
        public static FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of FuelSim
        private final CommandXboxController controller = new CommandXboxController(0);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        
        public RobotContainer() {
                
                led = new Led();
                DriverStation.silenceJoystickConnectionWarning(true);
                switch (Constants.currentMode) {
                        case REAL:
                                index = new Index(new IndexIOSpark());
                                flywheel = new Flywheel(new FlywheelIOSparkFlex());

                                kicker = new Kicker(new KickerIOSpark());
                                intake = new Intake(new IntakeIOSpark());
                                superstructure = new Superstructure(intake, flywheel, kicker, index);
                                LedSubsystem ledsub = new LedSubsystem(led, superstructure);                                break;

                        case SIM:
                                kicker = new Kicker(new KickerIOSpark());
                                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                                intake = new Intake(new IntakeIOSim());
                                index = new Index(new IndexIOSpark());
                                
                                superstructure = new Superstructure(intake, flywheel, kicker, index);
                                ledsub = new LedSubsystem(led, superstructure);
                                configureFuelSim();
                                break;

                        default:
                                kicker = new Kicker(new KickerIO() {
                                });
                                intake = new Intake(new IntakeIO() {
                                });
                                index = new Index(new IndexIO() {
                                });
                                flywheel = new Flywheel(new FlywheelIO() {
                                });
                                ledsub = new LedSubsystem(led, superstructure);
                                break;
                }
                NamedCommands.registerCommand("intake",
                                Commands.runOnce(() -> intake.requestState(SystemState.INTAKING), intake));
                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                autoChooser.addOption("pathplanner oto", new PathPlannerAuto("Example Auto"));

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a
         * {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
         * and then passing it to a
         * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                led.setStaticColor(Color.kBlue);

                controller.button(1).onTrue(superstructure.intakeCommand());

                controller.button(2).onTrue(superstructure.shootCommand());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public void configureFuelSim() {

                fuelSim.setSubticks(5); // sets the number of physics iterations to perform per 20ms loop. Default = 5
                fuelSim.enableAirResistance(); // an additional drag force will be applied to fuel in physics update
                                               // step

                fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone
                fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)

        }

}
