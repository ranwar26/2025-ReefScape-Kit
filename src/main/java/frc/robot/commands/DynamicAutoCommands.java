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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmControlCommands.ArmPosition;
import frc.robot.commands.ArmControlCommands.ArmSystem;
import frc.robot.commands.AutoDriveCommands.ReefSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;

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

  private static LoggedDashboardChooser<Integer> endAfterCycle = new LoggedDashboardChooser<>(
    networkKeyPrefix + "End after cycle");

  private static LoggedDashboardChooser<Pose2d> endingPose = new LoggedDashboardChooser<>(
    networkKeyPrefix + "Ending Pose");

  private static Drive drive;
  private static Pivot pivot;
  private static Elevator elevator;
  private static Wrist wrist;
  private static Intake intake;

  /**  
   * Sets up the subsystem and dashboard choosers for the Dynamic Auto system
   * 
   * @param drive the drive subsystem
   * @param pivot the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @param intake the intake subsystem
   */
  public static void setupDynamicAuto(Drive drive, Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    DynamicAutoCommands.drive = drive;
    DynamicAutoCommands.pivot = pivot;
    DynamicAutoCommands.elevator = elevator;
    DynamicAutoCommands.wrist = wrist;
    DynamicAutoCommands.intake = intake;

    chooserSetup();

  }

  /** 
   * Assembles the dashboard chooser's data into a command with arm and drive control. NOTE: calling this method a
   * second time will throw an error, but there is no plan to fix this, as an auto command should only be call once.
   * 
   * @return The command with this given logic.
   */
  public static Command buildDynamicAuto() {

    SequentialCommandGroup primaryCommandGroup = new SequentialCommandGroup();

    int totalCycles = endAfterCycle.get();

    primaryCommandGroup.addCommands(Commands.runOnce(
        () -> {
          drive.resetOdometry(startingPose.get());
        }));

    primaryCommandGroup.addCommands(getCycle(firstReefSide.get(), firstReefLevel.get(), firstCoralStation.get(), totalCycles));
    totalCycles--;
    primaryCommandGroup.addCommands(getCycle(secondReefSide.get(), secondReefLevel.get(), secondCoralStation.get(), totalCycles));
    totalCycles--;
    primaryCommandGroup.addCommands(getCycle(thirdReefSide.get(), thirdReefLevel.get(), thirdCoralStation.get(), totalCycles));
    totalCycles--;

    return primaryCommandGroup.andThen(
      AutoDriveCommands.pathFindToPose(endingPose.get() == null ? () -> drive.getPose() : () -> endingPose.get(), constraints, 0)
      .alongWith(ArmControlCommands.armHoldAtCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL).withTimeout(0.0)
        .andThen(ArmControlCommands.armDownCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION)))
    ).withName("dynamicAuto");
  }

  /**
   * Converters the given reef side, reef level, and coral station of a cycle into a command which contains the
   * auto drive, arm control, and intake movements.
   * 
   * @param reefSide the target reef side
   * @param reefLevel the target reef level
   * @param coralStation the target coral station (true = left)
   * @param cyclesLeft How many more cycle are need.
   * @return The command with the given logic
   */
  private static Command getCycle(ReefSide reefSide, ArmPosition reefLevel, Boolean coralStation, int cyclesLeft) {

    if(cyclesLeft <= 0)
      return Commands.waitSeconds(0.0);

    if(reefSide == null || reefLevel == null || coralStation == null) {
        Elastic.sendNotification(new Notification(Notification.NotificationLevel.ERROR, "AUTO CYCLE FAIL", "A dynamic auto cycle part was set as null"));
        return Commands.waitSeconds(0.0);
    }

    SequentialCommandGroup primaryCommandGroup = new SequentialCommandGroup();

    // Moves to the chosen reef side
    primaryCommandGroup.addCommands(
        AutoDriveCommands
        .pathFindToReef(drive, reefSide, constraints, false)
        .deadlineFor(ArmControlCommands
            .armDownCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION)
            .deadlineFor(IntakeCommands
                .intakeRun(intake, () -> 0.0)
            )
            .andThen(ArmControlCommands
            .armHoldAtCommand(pivot, elevator, wrist, ArmPosition.HOME, ArmSystem.ELEVATOR, ArmSystem.WRIST))
            .alongWith(ArmControlCommands
            .armUpCommand(pivot, elevator, wrist, reefLevel, ArmSystem.PIVOT))
            
        )
    );

    // Scores on the chosen reef level
    primaryCommandGroup.addCommands(
        ArmControlCommands
        .armUpCommand(pivot, elevator, wrist, reefLevel, ArmSystem.ALL)
        .deadlineFor(AutoDriveCommands
            .pathFindToReef(drive, reefSide, constraints, true)
        )
        .andThen(IntakeCommands
            .intakeRun(intake, () -> 1.0)
            .withTimeout(0.5)
            .deadlineFor(ArmControlCommands
                .armHoldAtCommand(pivot, elevator, wrist, reefLevel, ArmSystem.ALL)
            )
        )
    );

    // Moves to the chosen coral station
    primaryCommandGroup.addCommands(
        AutoDriveCommands.
            pathFindToCoralStation(drive, coralStation, constraints, false)
            .deadlineFor(ArmControlCommands
                .armDownCommand(pivot, elevator, wrist, reefLevel)
                .alongWith(IntakeCommands
                    .intakeRun(intake, () -> 0.0)
                )
        )
    );

    // Grabs coral out of the station
    primaryCommandGroup.addCommands(
        ArmControlCommands
        .armUpCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL)
        .deadlineFor(AutoDriveCommands
            .pathFindToCoralStation(drive, coralStation, constraints, true)
        )
        .andThen(IntakeCommands
            .intakeRun(intake, () -> -1.0)
            .withTimeout(1.0)
            .deadlineFor(ArmControlCommands
                .armHoldAtCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL)
            )
        )
    );


    return primaryCommandGroup;
  }

  /**
   * Adds all the dashboard choosers and their default, selectable options
   */
  public static void chooserSetup() {

    startingPose.addDefaultOption("Center", FieldPoses.center);
    startingPose.addOption("Left Wall Cage", FieldPoses.leftWallCage);
    startingPose.addOption("Left Center Cage", FieldPoses.leftCenterCage);
    startingPose.addOption("Left Post Cage", FieldPoses.leftPostCage);
    startingPose.addOption("Right Wall Cage", FieldPoses.rightWallCage);
    startingPose.addOption("Right Center Cage", FieldPoses.rightCenterCage);
    startingPose.addOption("Right Post Cage", FieldPoses.rightPostCage);

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

    endAfterCycle.addDefaultOption("Three", 3);
    endAfterCycle.addOption("Two", 2);
    endAfterCycle.addOption("One", 1);
    endAfterCycle.addOption("Zero", 0);

    endingPose.addDefaultOption("No end pose", null);
    endingPose.addOption("Center", FieldPoses.center);
    endingPose.addOption("Left Wall Cage", FieldPoses.leftWallCage);
    endingPose.addOption("Left Center Cage", FieldPoses.leftCenterCage);
    endingPose.addOption("Left Post Cage", FieldPoses.leftPostCage);
    endingPose.addOption("Right Wall Cage", FieldPoses.rightWallCage);
    endingPose.addOption("Right Center Cage", FieldPoses.rightCenterCage);
    endingPose.addOption("Right Post Cage", FieldPoses.rightPostCage);

  }

  public static class FieldPoses {

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
