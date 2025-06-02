// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ArmControlCommands.ArmPosition;
import frc.robot.commands.ArmControlCommands.ArmSystem;
import frc.robot.commands.AutoDriveCommands.ReefSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class DynamicAutoCommands {

  public static final String networkKeyPrefix = "Dynamic Auto/";

  public static final PathConstraints constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec,
      DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);

  private static LoggedDashboardChooser<Pose2d> startingPose = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Starting Pose");

  private static LoggedDashboardChooser<ReefSide> firstReefSide = new LoggedDashboardChooser<>(
      networkKeyPrefix + "First Reef Side");
  private static LoggedDashboardChooser<ArmPosition> firstReefLevel = new LoggedDashboardChooser<>(
      networkKeyPrefix + "First Reef Level");
  private static LoggedDashboardChooser<Boolean> firstCoralStation = new LoggedDashboardChooser<>(
      networkKeyPrefix + "First Coral Station");

  private static LoggedDashboardChooser<ReefSide> secondReefSide = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Second Reef Side");
  private static LoggedDashboardChooser<ArmPosition> secondReefLevel = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Second Reef Level");
  private static LoggedDashboardChooser<Boolean> secondCoralStation = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Second Coral Station");

  private static LoggedDashboardChooser<ReefSide> thirdReefSide = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Third Reef Side");
  private static LoggedDashboardChooser<ArmPosition> thirdReefLevel = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Third Reef Level");
  private static LoggedDashboardChooser<Boolean> thirdCoralStation = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Third Coral Station");

  private static Drive drive;
  private static Pivot pivot;
  private static Elevator elevator;
  private static Wrist wrist;
  private static Intake intake;

  public static void setupDynamicAuto(Drive drive, Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    DynamicAutoCommands.drive = drive;
    DynamicAutoCommands.pivot = pivot;
    DynamicAutoCommands.elevator = elevator;
    DynamicAutoCommands.wrist = wrist;
    DynamicAutoCommands.intake = intake;

    chooserSetup();

  }

  public static Command buildDynamicAuto() {

    SequentialCommandGroup primaryCommandGroup = new SequentialCommandGroup();

    primaryCommandGroup.addCommands(Commands.runOnce(
        () -> {
          drive.resetOdometry(startingPose.get());
        }));

    primaryCommandGroup.addCommands(getCycle(firstReefSide.get(), firstReefLevel.get(), firstCoralStation.get()));
    primaryCommandGroup.addCommands(getCycle(secondReefSide.get(), secondReefLevel.get(), secondCoralStation.get()));
    primaryCommandGroup.addCommands(getCycle(thirdReefSide.get(), thirdReefLevel.get(), thirdCoralStation.get()));

    return primaryCommandGroup.withName("dynamicAuto");
  }

  private static Command getCycle(ReefSide reefSide, ArmPosition reefLevel, Boolean coralStation) {

    SequentialCommandGroup primaryCommandGroup = new SequentialCommandGroup();

    // Moves to the chosen reef side
    primaryCommandGroup.addCommands(
        AutoDriveCommands.pathFindToReef(drive, reefSide, constraints, false).deadlineFor(
            ArmControlCommands.armDownCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION).andThen(ArmControlCommands.armUpCommand(pivot, elevator, wrist, reefLevel, ArmSystem.PIVOT))
        )
    );

    // Scores on the chosen reef level
    primaryCommandGroup.addCommands(
        ArmControlCommands.armUpCommand(pivot, elevator, wrist, reefLevel, ArmSystem.ALL).deadlineFor(
            AutoDriveCommands.pathFindToReef(drive, reefSide, constraints, true)
        ),
        Commands.waitSeconds(0.5).deadlineFor(
            ArmControlCommands.armHoldAtCommand(pivot, elevator, wrist, reefLevel, ArmSystem.ALL).alongWith(IntakeCommands.intakeRun(intake, () -> 1.0))
        )
    );

    // Moves to the chosen coral station
    primaryCommandGroup.addCommands(
        AutoDriveCommands.pathFindToCoralStation(drive, coralStation, constraints, false).deadlineFor(
            ArmControlCommands.armDownCommand(pivot, elevator, wrist, reefLevel).andThen(ArmControlCommands.armUpCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.PIVOT))
        )
    );

    // Grabs coral out of the station
    primaryCommandGroup.addCommands(
        ArmControlCommands.armUpCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL).deadlineFor(
            AutoDriveCommands.pathFindToCoralStation(drive, coralStation, constraints, true)
        ),
        Commands.waitSeconds(0.5).deadlineFor(
            ArmControlCommands.armHoldAtCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL).alongWith(IntakeCommands.intakeRun(intake, () -> -1.0))
        )
    );


    return primaryCommandGroup;
  }

  public static void chooserSetup() {

    startingPose.addDefaultOption("Center", StartingPoses.center);
    startingPose.addOption("Left Wall Cage", StartingPoses.leftWallCage);
    startingPose.addOption("Left Center Cage", StartingPoses.leftCenterCage);
    startingPose.addOption("Left Post Cage", StartingPoses.leftPostCage);
    startingPose.addOption("Right Wall Cage", StartingPoses.rightWallCage);
    startingPose.addOption("Right Center Cage", StartingPoses.rightCenterCage);
    startingPose.addOption("Right Post Cage", StartingPoses.rightPostCage);

    // ############ FIRST CYCLE ############
    firstReefSide.addDefaultOption("Please Pick One", null);
    firstReefSide.addOption("Front", ReefSide.FRONT);
    firstReefSide.addOption("Front Left", ReefSide.FRONT_LEFT);
    firstReefSide.addOption("Front Right", ReefSide.FRONT_RIGHT);
    firstReefSide.addOption("Back", ReefSide.BACK);
    firstReefSide.addOption("Back Left", ReefSide.BACK_LEFT);
    firstReefSide.addOption("Back Right", ReefSide.BACK_RIGHT);

    firstReefLevel.addDefaultOption("Please Pick One", null);
    firstReefLevel.addOption("Level 2", ArmPosition.LEVEL2);
    firstReefLevel.addOption("Level 3", ArmPosition.LEVEL3);
    firstReefLevel.addOption("Level 4", ArmPosition.LEVEL4);

    firstCoralStation.addDefaultOption("Please Pick One", null);
    firstCoralStation.addOption("Left Side", true);
    firstCoralStation.addOption("Right Side", false);

    // ############ SECOND CYCLE ############
    secondReefSide.addDefaultOption("Please Pick One", null);
    secondReefSide.addOption("Front", ReefSide.FRONT);
    secondReefSide.addOption("Front Left", ReefSide.FRONT_LEFT);
    secondReefSide.addOption("Front Right", ReefSide.FRONT_RIGHT);
    secondReefSide.addOption("Back", ReefSide.BACK);
    secondReefSide.addOption("Back Left", ReefSide.BACK_LEFT);
    secondReefSide.addOption("Back Right", ReefSide.BACK_RIGHT);

    secondReefLevel.addDefaultOption("Please Pick One", null);
    secondReefLevel.addOption("Level 2", ArmPosition.LEVEL2);
    secondReefLevel.addOption("Level 3", ArmPosition.LEVEL3);
    secondReefLevel.addOption("Level 4", ArmPosition.LEVEL4);

    secondCoralStation.addDefaultOption("Please Pick One", null);
    secondCoralStation.addOption("Left Side", true);
    secondCoralStation.addOption("Right Side", false);

    // ############ THIRD CYCLE ############
    thirdReefSide.addDefaultOption("Please Pick One", null);
    thirdReefSide.addOption("Front", ReefSide.FRONT);
    thirdReefSide.addOption("Front Left", ReefSide.FRONT_LEFT);
    thirdReefSide.addOption("Front Right", ReefSide.FRONT_RIGHT);
    thirdReefSide.addOption("Back", ReefSide.BACK);
    thirdReefSide.addOption("Back Left", ReefSide.BACK_LEFT);
    thirdReefSide.addOption("Back Right", ReefSide.BACK_RIGHT);

    thirdReefLevel.addDefaultOption("Please Pick One", null);
    thirdReefLevel.addOption("Level 2", ArmPosition.LEVEL2);
    thirdReefLevel.addOption("Level 3", ArmPosition.LEVEL3);
    thirdReefLevel.addOption("Level 4", ArmPosition.LEVEL4);

    thirdCoralStation.addDefaultOption("Please Pick One", null);
    thirdCoralStation.addOption("Left Side", true);
    thirdCoralStation.addOption("Right Side", false);

  }

  public static class StartingPoses {

    /** Blue side */
    public static final Pose2d leftWallCage_BlueSide = new Pose2d(7.1, 7.3, new Rotation2d());
    public static final Pose2d leftCenterCage_BlueSide = new Pose2d(7.1, 6.2, new Rotation2d());
    public static final Pose2d leftPostCage_BlueSide = new Pose2d(7.1, 5.1, new Rotation2d());
    public static final Pose2d center_BlueSide = new Pose2d(7.1, 4.0, new Rotation2d());
    public static final Pose2d rightWallCage_BlueSide = new Pose2d(7.1, 0.8, new Rotation2d());
    public static final Pose2d rightCenterCage_BlueSide = new Pose2d(7.1, 1.9, new Rotation2d());
    public static final Pose2d rightPostCage_BlueSide = new Pose2d(7.1, 3.0, new Rotation2d());

    /** Red Side */
    public static final Pose2d leftWallCage_RedSide = new Pose2d(10.4, 0.8, new Rotation2d());
    public static final Pose2d leftCenterCage_RedSide = new Pose2d(10.4, 1.9, new Rotation2d());
    public static final Pose2d leftPostCage_RedSide = new Pose2d(10.4, 3.0, new Rotation2d());
    public static final Pose2d center_RedSide = new Pose2d(10.4, 4.0, new Rotation2d());
    public static final Pose2d rightWallCage_RedSide = new Pose2d(10.4, 7.3, new Rotation2d());
    public static final Pose2d rightCenterCage_RedSide = new Pose2d(10.4, 6.2, new Rotation2d());
    public static final Pose2d rightPostCage_RedSide = new Pose2d(10.4, 5.1, new Rotation2d());

    /** Our team side */
    public static final Pose2d leftWallCage = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? leftWallCage_BlueSide
        : leftWallCage_RedSide;
    public static final Pose2d leftCenterCage = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? leftCenterCage_BlueSide
        : leftCenterCage_RedSide;
    public static final Pose2d leftPostCage = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? leftPostCage_BlueSide
        : leftPostCage_RedSide;
    public static final Pose2d center = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? center_BlueSide
        : center_RedSide;
    public static final Pose2d rightWallCage = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? rightWallCage_BlueSide
        : rightWallCage_RedSide;
    public static final Pose2d rightCenterCage = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? rightCenterCage_BlueSide
        : rightCenterCage_RedSide;
    public static final Pose2d rightPostCage = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? rightPostCage_BlueSide
        : rightPostCage_RedSide;
  }

}
