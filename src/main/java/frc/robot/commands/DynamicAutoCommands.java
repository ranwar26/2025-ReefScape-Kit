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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
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

  private static LoggedDashboardChooser<Integer> firstReefSide = new LoggedDashboardChooser<>(
      networkKeyPrefix + "First Reef Side");
  private static LoggedDashboardChooser<Command> firstReefLevel = new LoggedDashboardChooser<>(
      networkKeyPrefix + "First Reef Level");
  private static LoggedDashboardChooser<Boolean> firstCoralStation = new LoggedDashboardChooser<>(
      networkKeyPrefix + "First Coral Station");

  private static LoggedDashboardChooser<Integer> secondReefSide = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Second Reef Side");
  private static LoggedDashboardChooser<Command> secondReefLevel = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Second Reef Level");
  private static LoggedDashboardChooser<Boolean> secondCoralStation = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Second Coral Station");

  private static LoggedDashboardChooser<Integer> thirdReefSide = new LoggedDashboardChooser<>(
      networkKeyPrefix + "Third Reef Side");
  private static LoggedDashboardChooser<Command> thirdReefLevel = new LoggedDashboardChooser<>(
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

  public static Command getDynamicAuto() {

    SequentialCommandGroup primaryCommandGroup = new SequentialCommandGroup();

    primaryCommandGroup.addCommands(Commands.runOnce(
        () -> {
          drive.resetOdometry(startingPose.get());
        }));

    primaryCommandGroup.addCommands(getCycle(firstReefSide, firstReefLevel, firstCoralStation));
    primaryCommandGroup.addCommands(getCycle(secondReefSide, secondReefLevel, secondCoralStation));
    primaryCommandGroup.addCommands(getCycle(thirdReefSide, thirdReefLevel, thirdCoralStation));

    return primaryCommandGroup.withName("dynamicAuto");
  }

  private static Command getCycle(LoggedDashboardChooser<Integer> reefSide, LoggedDashboardChooser<Command> reefLevel,
      LoggedDashboardChooser<Boolean> coralStation) {

    SequentialCommandGroup primaryCommandGroup = new SequentialCommandGroup();

    // Moves to the chosen reef side
    if (reefSide.get() != null)
      primaryCommandGroup.addCommands(AutoDriveCommands.pathFindToReef(drive, reefSide.get(), constraints)
          .deadlineFor(ArmControlCommandGroups.homeCommandGroup(pivot, elevator, wrist, false)));

    // Scores on the chosen reef level
    if (reefLevel.get() != null)
      primaryCommandGroup.addCommands(new SequentialCommandGroup(
          reefLevel.get(),
          new ParallelDeadlineGroup(
              new WaitCommand(0.5),
              IntakeCommands.intakeRun(intake, () -> 1.0),
              ArmControlCommandGroups.holdCommandGroup(pivot, elevator, wrist)),
          IntakeCommands.intakeRun(intake, () -> 0.0).withTimeout(0.0)));

    // Moves and grabs a coral out of the chosen coral station
    if (coralStation.get() != null)
      primaryCommandGroup.addCommands(new SequentialCommandGroup(
          AutoDriveCommands.pathFindToCoralStation(coralStation.get(), constraints)
              .deadlineFor(ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)),
          ArmControlCommandGroups.coralStationUpCommandGroup(pivot, elevator, wrist),
          new ParallelDeadlineGroup(
              new WaitCommand(1.0),
              IntakeCommands.intakeRun(intake, () -> -1.0),
              ArmControlCommandGroups.holdCommandGroup(pivot, elevator, wrist)),
          IntakeCommands.intakeRun(intake, () -> 0.0).withTimeout(0.0)));

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
    firstReefSide.addDefaultOption("SKIP", null);
    firstReefSide.addOption("Front", 1);
    firstReefSide.addOption("Front Left", 2);
    firstReefSide.addOption("Front Right", 3);
    firstReefSide.addOption("Back", 4);
    firstReefSide.addOption("Back Left", 5);
    firstReefSide.addOption("Back Right", 6);

    firstReefLevel.addDefaultOption("SKIP", null);
    firstReefLevel.addOption("Level 2", ArmControlCommandGroups.level2UpCommandGroup(pivot, elevator, wrist));
    firstReefLevel.addOption("Level 3", ArmControlCommandGroups.level3UpCommandGroup(pivot, elevator, wrist));
    firstReefLevel.addOption("Level 4", ArmControlCommandGroups.level4UpCommandGroup(pivot, elevator, wrist));

    firstCoralStation.addDefaultOption("SKIP", null);
    firstCoralStation.addOption("Left Side", true);
    firstCoralStation.addOption("Right Side", false);

    // ############ SECOND CYCLE ############
    secondReefSide.addDefaultOption("SKIP", null);
    secondReefSide.addOption("Front", 1);
    secondReefSide.addOption("Front Left", 2);
    secondReefSide.addOption("Front Right", 3);
    secondReefSide.addOption("Back", 4);
    secondReefSide.addOption("Back Left", 5);
    secondReefSide.addOption("Back Right", 6);

    secondReefLevel.addDefaultOption("SKIP", null);
    secondReefLevel.addOption("Level 2", ArmControlCommandGroups.level2UpCommandGroup(pivot, elevator, wrist));
    secondReefLevel.addOption("Level 3", ArmControlCommandGroups.level3UpCommandGroup(pivot, elevator, wrist));
    secondReefLevel.addOption("Level 4", ArmControlCommandGroups.level4UpCommandGroup(pivot, elevator, wrist));

    secondCoralStation.addDefaultOption("SKIP", null);
    secondCoralStation.addOption("Left Side", true);
    secondCoralStation.addOption("Right Side", false);

    // ############ THIRD CYCLE ############
    thirdReefSide.addDefaultOption("SKIP", null);
    thirdReefSide.addOption("Front", 1);
    thirdReefSide.addOption("Front Left", 2);
    thirdReefSide.addOption("Front Right", 3);
    thirdReefSide.addOption("Back", 4);
    thirdReefSide.addOption("Back Left", 5);
    thirdReefSide.addOption("Back Right", 6);

    thirdReefLevel.addDefaultOption("SKIP", null);
    thirdReefLevel.addOption("Level 2", ArmControlCommandGroups.level2UpCommandGroup(pivot, elevator, wrist));
    thirdReefLevel.addOption("Level 3", ArmControlCommandGroups.level3UpCommandGroup(pivot, elevator, wrist));
    thirdReefLevel.addOption("Level 4", ArmControlCommandGroups.level4UpCommandGroup(pivot, elevator, wrist));

    thirdCoralStation.addDefaultOption("SKIP", null);
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
