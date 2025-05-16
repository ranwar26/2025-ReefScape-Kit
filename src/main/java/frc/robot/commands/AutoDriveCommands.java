// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
public class AutoDriveCommands {

    // For Demoing use only
    public static Command autoDriveAndScore(Drive drive, Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {
    
        SequentialCommandGroup primaryCommand = new SequentialCommandGroup();

        for(int i = 0; i < 50; i++) {

            // ################### GOING TO CORAL STATION ###################

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                PathplannerOnFlyCommands.pathFindToCoralStation(Math.random() < 0.5, null),
                ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)
                    .andThen(ArmControlCommandGroups.homeCommandGroup(pivot, elevator, wrist)
                    ),
                IntakeCommands.intakeRun(intake, () -> 0.0)
            ));

            primaryCommand.addCommands(ArmControlCommandGroups.coralStationUpCommandGroup(pivot, elevator, wrist));

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(0.5), // Command group waits on this
                IntakeCommands.intakeRun(intake, () -> 1.0),
                ArmControlCommandGroups.holdCommandGroup(pivot, elevator, wrist)
            ));

            
            // ################### GOING TO REEF ###################

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                PathplannerOnFlyCommands.pathFindToReef((int) (Math.random() * 6) + 1, null),
                ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)
                    .andThen(ArmControlCommandGroups.homeCommandGroup(pivot, elevator, wrist)
                    ),
                IntakeCommands.intakeRun(intake, () -> 0.0)
            ));

            Command targetLevelCommand = null;
            switch ((int) (Math.random() * 3) + 2) {
                case 2:
                    targetLevelCommand = ArmControlCommandGroups.Level2UpCommandGroup(pivot, elevator, wrist);
                    break;
                case 3:
                    targetLevelCommand = ArmControlCommandGroups.Level3UpCommandGroup(pivot, elevator, wrist);
                    break;
                case 4:
                    targetLevelCommand = ArmControlCommandGroups.Level4UpCommandGroup(pivot, elevator, wrist);
                    break;
            }

            primaryCommand.addCommands(targetLevelCommand);

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(0.5), // Command group waits on this
                IntakeCommands.intakeRun(intake, () -> -1.0),
                ArmControlCommandGroups.holdCommandGroup(pivot, elevator, wrist)
            ));

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> Math.abs(
                    elevator.getCurrentLength() - ElevatorConstants.kHomeLength) < ElevatorConstants.kLengthErrorAllowed
                ),
                ArmControlCommandGroups.retractCommandGroup(pivot, elevator, wrist)
            ));
        }

        return primaryCommand;
    }
}
