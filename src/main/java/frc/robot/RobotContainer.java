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

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MechanismCommands;
import frc.robot.commands.PathplannerOnFlyCommands;
import frc.robot.commands.PivotCommands;
import frc.robot.commands.ArmControlCommandGroups;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
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
	private final Vision vision;
	private final Pivot pivot;
	private final Elevator elevator;
	private final Wrist wrist;
	private final Intake intake;

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
						new ModuleIOSpark(3));
				vision = new Vision(
					drive::addVisionMeasurement,
					new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
					new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

				pivot = new Pivot(new PivotIOReal());
				elevator = new Elevator(new ElevatorIOReal());
				wrist = new Wrist(new WristIOReal());
				intake = new Intake(new IntakeIOReal());

				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				drive = new Drive(
						new GyroIO() {},
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim());
				// vision = new Vision(drive::addVisionMeasurement,
				// new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
				// new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
				vision = new Vision(drive::addVisionMeasurement,
				new VisionIO() {},
				new VisionIO() {});

				pivot = new Pivot(new PivotIOSim());
				elevator = new Elevator(new ElevatorIOSim());
				wrist = new Wrist(new WristIOSim());
				intake = new Intake(new IntakeIOSim());

				break;

			default:
				// Replayed robot, disable IO implementations
				drive = new Drive(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {});
				vision = new Vision(drive::addVisionMeasurement,
				new VisionIO() {},
				new VisionIO() {});

				pivot = new Pivot(new PivotIO() {});
				elevator = new Elevator(new ElevatorIO() {});
				wrist = new Wrist(new WristIO() {});
				intake = new Intake(new IntakeIO() {});

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
				() -> MathUtil.applyDeadband(controller.getRightTriggerAxis(), OIConstants.kDriveDeadband),
				() -> MathUtil.applyDeadband(-controller.getLeftY(), OIConstants.kDriveDeadband),
				() -> MathUtil.applyDeadband(-controller.getLeftX(), OIConstants.kDriveDeadband),
				() -> MathUtil.applyDeadband(-controller.getRightY(), OIConstants.kDriveDeadband),
				() -> MathUtil.applyDeadband(-controller.getRightX(), OIConstants.kDriveDeadband)));

		// Default command for each subsystem
		intake.setDefaultCommand(IntakeCommands.intakeRun(intake, () -> 0.0));

		wrist.setDefaultCommand(WristCommands.wristToHome(wrist, false));

		elevator.setDefaultCommand(ElevatorCommands.elevatorToHome(elevator, false));

		pivot.setDefaultCommand(PivotCommands.pivotToHome(pivot, false));

		// Starts the arm mechanism for sim and comp matches
		MechanismCommands.mechanismRun(pivot, elevator, wrist, intake).ignoringDisable(true).schedule();

		Supplier<Translation2d> robot = () -> drive.getPose().getTranslation();
		Pathfinding.setDynamicObstacles(DriveConstants.opposingCages, robot.get());

		// Configure the button bindings
		switch (Constants.currentMode) {
			case SIM:
				configureREALButtonBindings();
				configureSIMButtonBindings(); // Add/Overwrite sim bindings
				break;
			default:
				configureREALButtonBindings();
				break;
		}

		PathfindingCommand.warmupCommand().schedule();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a
	 * {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
	 * and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureREALButtonBindings() {

		// Point all wheel towards the center of the robot.
		controller.x().whileTrue(Commands.runOnce(drive::stopWithX, drive));

		controller.start().onTrue(Commands.runOnce(
				() -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
				drive).ignoringDisable(true));

		controller.leftTrigger().onTrue(IntakeCommands.intakeRun(intake, () -> controller.getLeftTriggerAxis()));

		controller.a().whileTrue(ArmControlCommandGroups.Level2UpCommandGroup(pivot, elevator, wrist, false));
		controller.b().whileTrue(ArmControlCommandGroups.Level3UpCommandGroup(pivot, elevator, wrist, false));
		controller.y().whileTrue(ArmControlCommandGroups.Level4UpCommandGroup(pivot, elevator, wrist, false));

		controller.a().onFalse(ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist, true));
		controller.b().onFalse(ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist, true));
		controller.y().onFalse(ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist, true));

		controller.leftBumper().onTrue(PathplannerOnFlyCommands.pathFindToCoralStation(true, null));
		controller.rightBumper().onTrue(PathplannerOnFlyCommands.pathFindToCoralStation(false, null));

		controller.povDown().onTrue(PathplannerOnFlyCommands.pathFindToReef(1,null));
		controller.povDownLeft().onTrue(PathplannerOnFlyCommands.pathFindToReef(2,null));
		controller.povLeft().onTrue(PathplannerOnFlyCommands.pathFindToReef(2,null));
		controller.povDownRight().onTrue(PathplannerOnFlyCommands.pathFindToReef(3, null));
		controller.povRight().onTrue(PathplannerOnFlyCommands.pathFindToReef(3, null));
		controller.povUp().onTrue(PathplannerOnFlyCommands.pathFindToReef(4, null));
		controller.povUpLeft().onTrue(PathplannerOnFlyCommands.pathFindToReef(5, null));
		controller.povUpRight().onTrue(PathplannerOnFlyCommands.pathFindToReef(6, null));

		controller.back().onTrue(PathplannerOnFlyCommands.pathFindToPose(
			() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? new Pose2d(15.5, 4.0, new Rotation2d(Math.PI)) : new Pose2d(2.0, 4.0, new Rotation2d()),
		PathConstraints.unlimitedConstraints(12.0), 0));
		
		// Used to stop any path finding happing
		controller.leftStick().whileTrue(Commands.runOnce(() -> {}, drive));

		// Used for demoing robot
		controller.rightStick().onTrue(AutoDriveCommands.autoDriveAndScore(drive, pivot, elevator, wrist, intake));
	}

	public void configureSIMButtonBindings() {

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

}
