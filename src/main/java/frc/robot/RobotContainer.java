// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MechanismCommands;
import frc.robot.commands.PivotCommands;
import frc.robot.commands.ReefLevelsCommandGroups;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import frc.robot.subsystems.wrist.WristIOSim;

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
    private final Drive drive;
    private final Pivot pivot;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Intake intake;
    private final Vision vision;
    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                        (pose) -> {
                        });
                pivot = new Pivot(new PivotIOReal());
                elevator = new Elevator(new ElevatorIOReal());
                wrist = new Wrist(new WristIOReal());
                intake = new Intake(new IntakeIOReal());

                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                pivot = new Pivot(new PivotIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                wrist = new Wrist(new WristIOSim());
                intake = new Intake(new IntakeIOSim());

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1,
                                driveSimulation::getSimulatedDriveTrainPose));

                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        (pose) -> {
                        });

                pivot = new Pivot(new PivotIO() {
                });
                elevator = new Elevator(new ElevatorIO() {
                });
                wrist = new Wrist(new WristIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                vision = new Vision(
                        drive,
                        new VisionIO() {
                        },
                        new VisionIO() {
                        });

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> -controller.getRightY(),
                () -> -controller.getRightX()));

        intake.setDefaultCommand(IntakeCommands.intakeRun(intake, () -> 0.0));

        wrist.setDefaultCommand(WristCommands.wristToHome(wrist));

        elevator.setDefaultCommand(ElevatorCommands.wristToHome(elevator));

        pivot.setDefaultCommand(PivotCommands.pivotToHome(pivot));

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

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual
                                                                                          // robot pose during
                                                                                          // simulation
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero
                                                                                                             // gyro

        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        controller.leftTrigger().onTrue(IntakeCommands.intakeRun(
                intake,
                () -> controller.getLeftTriggerAxis()));

        controller.leftBumper().whileTrue(
                ReefLevelsCommandGroups.Level2UpCommandGroup(pivot, elevator, wrist, intake));

        controller.rightBumper().whileTrue(
                ReefLevelsCommandGroups.Level3UpCommandGroup(pivot, elevator, wrist, intake));

        controller.y().whileTrue(
                ReefLevelsCommandGroups.Level4UpCommandGroup(pivot, elevator, wrist, intake));

        controller.povUp().onTrue(MechanismCommands.mechanismRun(pivot, elevator, wrist, intake));

        controller.a().whileTrue(Commands.runOnce(
                () -> {
                    ReefscapeCoralOnField coral = new ReefscapeCoralOnField(new Pose2d(3.0, 4.0, new Rotation2d()));
                    SimulatedArena.getInstance().addGamePiece(coral);
                },
                drive));
        controller.b().whileTrue(Commands.runOnce(
                () -> {
                    ReefscapeAlgaeOnField algae = new ReefscapeAlgaeOnField(new Translation2d(3.0, 5.0));
                    SimulatedArena.getInstance().addGamePiece(algae);
                },
                drive));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
