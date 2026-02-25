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
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSparkFlex;
import frc.robot.subsystems.hood.Hood;
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
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
        public static Drive drive;
        @Getter
        public static Vision vision;
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
        @Getter
        public static Hood hood;
        @Getter
        public static Turret turret;
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
                LedSubsystem ledsub = new LedSubsystem(led, superstructure);
                DriverStation.silenceJoystickConnectionWarning(true);
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drive = new Drive(
                                                new GyroIOPigeon2(),
                                                new ModuleIOSpark(0),
                                                new ModuleIOSpark(1),
                                                new ModuleIOSpark(2),
                                                new ModuleIOSpark(3));

                                index = new Index(new IndexIOSpark());
                                turret = new Turret(new TurretIOSpark());

                                vision = new Vision(
                                                drive::addVisionMeasurement,
                                                new VisionIOPhotonVision(VisionConstants.camera1Name,
                                                                VisionConstants.robotToCamera1));
                                flywheel = new Flywheel(new FlywheelIOSparkFlex());

                                kicker = new Kicker(new KickerIOSpark());
                                intake = new Intake(new IntakeIOSpark());
                                superstructure = new Superstructure(intake, flywheel, kicker, hood,  turret,  drive, index);
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                }, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(),
                                                new ModuleIOSim());
                                vision = new Vision(
                                                drive::addVisionMeasurement,
                                                new VisionIOPhotonVisionSim(VisionConstants.camera1Name,
                                                                VisionConstants.robotToCamera1,
                                                                drive::getPose));

                                kicker = new Kicker(new KickerIOSpark());
                                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                                intake = new Intake(new IntakeIOSim());
                                index = new Index(new IndexIOSpark());
                                turret = new Turret(new TurretIOSim(fuelSim));
                                configureFuelSim();
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                }, new ModuleIO() {
                                                }, new ModuleIO() {
                                                }, new ModuleIO() {
                                                }, new ModuleIO() {
                                                });
                                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                                });

                                kicker = new Kicker(new KickerIO() {
                                });
                                intake = new Intake(new IntakeIO() {
                                });
                                index = new Index(new IndexIO() {
                                });
                                flywheel = new Flywheel(new FlywheelIO() {
                                });
                                turret = new Turret(new TurretIO() {
                                });
                                break;
                }
                NamedCommands.registerCommand("intake",
                                Commands.runOnce(() -> intake.requestState(SystemState.INTAKING), intake));
                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                autoChooser.addOption("pathplanner oto", new PathPlannerAuto("Example Auto"));
                // Set up SysId routines
                autoChooser.addOption("Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption("Drive Simple FF Characterization",
                                DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption("Drive SysId (Dynamic Forward)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                // Old bindings commented out for reference
                drive.setDefaultCommand(DriveCommands.joystickDrive(
                                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(),
                                () -> -controller.getRawAxis(2)));

                // Lock to 0° when A button is held
                controller
                                .a()
                                .whileTrue(DriveCommands.joystickDriveAtAngle(
                                                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(),
                                                () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

                // Reset gyro to 0° when B button is pressed
                controller
                                .b()
                                .onTrue(Commands.runOnce(
                                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                                                                new Rotation2d())),
                                                drive)
                                                .ignoringDisable(true));
                controller.x().onTrue(Commands.runOnce(() -> intake.requestState(SystemState.CLOSING), intake));

                controller.y().onTrue(Commands.runOnce(() -> intake.requestState(SystemState.INTAKING), intake));
                controller.leftBumper().whileTrue(Commands.runOnce(() -> turret.launchFuel(), turret).repeatedly());
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

                // Register a robot for collision with fuel
                fuelSim.registerRobot(
                                0.6218, // from left to right in meters
                                0.7239, // from front to back in meters
                                0.12, // from floor to top of bumpers in meters
                                () -> getDrive().getPose(), // Supplier<Pose2d> of robot pose
                                () -> getDrive().getFieldSpeeds()); // Supplier<ChassisSpeeds> of field-centric chassis
                                                                    // speeds

                // Register an intake to remove fuel from the field as a rectangular bounding
                // box
                fuelSim.registerIntake(
                                -0.59, -0.45, -0.27, 0.273, // robot-centric coordinates for bounding box in meters
                                () -> turret.canIntake() && intake.isOpened(), // (optional) BooleanSupplier for whether
                                                                               // the intake should be
                                // active at a given moment
                                () -> turret.intakeFuel()); // (optional) Runnable called whenever a fuel is intaked

                fuelSim.setSubticks(5); // sets the number of physics iterations to perform per 20ms loop. Default = 5
                fuelSim.enableAirResistance(); // an additional drag force will be applied to fuel in physics update
                                               // step

                fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone
                fuelSim.start(); // enables the simulation to run (updateSim must still be called periodically)

        }

}
