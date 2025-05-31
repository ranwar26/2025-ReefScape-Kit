// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class AutoScoreCommands {

    // For Demoing use only
    public static Command autoDriveAndScore(Drive drive, Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {
    
        SequentialCommandGroup primaryCommand = new SequentialCommandGroup();

        for(int i = 0; i < 50; i++) {

            // ################### GOING TO CORAL STATION ###################

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                AutoDriveCommands.pathFindToCoralStation(Math.random() < 0.5, null),
                ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)
                    .andThen(ArmControlCommandGroups.homeCommandGroup(pivot, elevator, wrist, false)
                    ),
                IntakeCommands.intakeRun(intake, () -> 0.0)
            ));

            primaryCommand.addCommands(ArmControlCommandGroups.coralStationUpCommandGroup(pivot, elevator, wrist));

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(1.0), // Command group waits on this
                IntakeCommands.intakeRun(intake, () -> 1.0),
                ArmControlCommandGroups.holdAtCoralStationCommandGroup(pivot, elevator, wrist)
            ));

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                IntakeCommands.intakeRun(intake, () -> 0.0),
                PivotCommands.pivotToHome(pivot, false),
                ElevatorCommands.elevatorHold(elevator),
                WristCommands.wristHold(wrist)
                ));

            
            // ################### GOING TO REEF ###################

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                AutoDriveCommands.pathFindToReef(drive, (int) (Math.random() * 6) + 1, null,true),
                ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)
                    .andThen(ArmControlCommandGroups.homeCommandGroup(pivot, elevator, wrist, false)
                    )
            ));

            Command targetLevelCommand = null;
            Command targetHoldLevelCommand = null;
            switch ((int) (Math.random() * 3) + 2) {
                case 2:
                    targetLevelCommand = ArmControlCommandGroups.level2UpCommandGroup(pivot, elevator, wrist);
                    targetHoldLevelCommand = ArmControlCommandGroups.holdAtLevel2CommandGroup(pivot, elevator, wrist);
                    break;
                case 3:
                    targetLevelCommand = ArmControlCommandGroups.level3UpCommandGroup(pivot, elevator, wrist);
                    targetHoldLevelCommand = ArmControlCommandGroups.holdAtLevel3CommandGroup(pivot, elevator, wrist);
                    break;
                case 4:
                    targetLevelCommand = ArmControlCommandGroups.level4UpCommandGroup(pivot, elevator, wrist);
                    targetHoldLevelCommand = ArmControlCommandGroups.holdAtLevel4CommandGroup(pivot, elevator, wrist);
                    break;
            }

            primaryCommand.addCommands(targetLevelCommand);

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(0.5), // Command group waits on this
                IntakeCommands.intakeRun(intake, () -> -1.0),
                targetHoldLevelCommand
            ));

            primaryCommand.addCommands(new ParallelCommandGroup(
                IntakeCommands.intakeRun(intake, () -> 0.0),
                ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)
            ).until(() -> Math.abs(elevator.getCurrentLength() - ElevatorConstants.kHomeLength) < ElevatorConstants.kLengthErrorAllowed));
        }

        return primaryCommand.withName("autoDriveAndScore");
    }
}
