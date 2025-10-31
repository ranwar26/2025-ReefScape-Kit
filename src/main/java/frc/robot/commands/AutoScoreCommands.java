// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmControlCommands.ArmPosition;
import frc.robot.commands.ArmControlCommands.ArmSystem;
import frc.robot.commands.AutoDriveCommands.ReefSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/**
 * This class is used for deming, so no docs (It's also outdated).
 */
public class AutoScoreCommands {

    // For Demoing use only
    public static Command autoDriveAndScore(Drive drive, Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {
    
        SequentialCommandGroup primaryCommand = new SequentialCommandGroup();

        for(int i = 0; i < 50; i++) {

            // ################### GOING TO CORAL STATION ###################

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                AutoDriveCommands.pathFindToCoralStation(drive, Math.random() < 0.5, null, true),
                ArmControlCommands.armDownCommand(pivot, elevator, wrist, null),
                IntakeCommands.intakeRun(intake, () -> 0.0)
            ));

            primaryCommand.addCommands(ArmControlCommands.armUpCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL));

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(1.0), // Command group waits on this
                IntakeCommands.intakeRun(intake, () -> 1.0),
                ArmControlCommands.armHoldAtCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION, ArmSystem.ALL)
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
                AutoDriveCommands.pathFindToReef(drive, getRandomSide(), null,true),
                ArmControlCommands.armDownCommand(pivot, elevator, wrist, ArmPosition.CORAL_STATION)
            ));

            ArmPosition targetLevel = null;
            switch ((int) (Math.random() * 3) + 2) {
                case 2:
                    targetLevel = ArmPosition.LEVEL2;
                    break;
                case 3:
                    targetLevel = ArmPosition.LEVEL3;
                    break;
                case 4:
                    targetLevel = ArmPosition.LEVEL4;
                    break;
            }

            primaryCommand.addCommands(ArmControlCommands.armUpCommand(pivot, elevator, wrist, targetLevel, ArmSystem.ALL));

            primaryCommand.addCommands(new ParallelDeadlineGroup(
                new WaitCommand(0.5), // Command group waits on this
                IntakeCommands.intakeRun(intake, () -> -1.0),
                ArmControlCommands.armHoldAtCommand(pivot, elevator, wrist, targetLevel, ArmSystem.ALL)
            ));

            primaryCommand.addCommands(new ParallelCommandGroup(
                IntakeCommands.intakeRun(intake, () -> 0.0),
                ArmControlCommands.armDownCommand(pivot, elevator, wrist, targetLevel)
            ).until(() -> Math.abs(elevator.getCurrentLength() - ElevatorConstants.kHomeLength) < ElevatorConstants.kLengthErrorAllowed));
        }

        return primaryCommand.withName("autoDriveAndScore");
    }

    public static ReefSide getRandomSide() {

        switch ((int) (Math.random() * 6.0)) {
            case 0:
                return ReefSide.FRONT;
            case 1:
                return ReefSide.FRONT_LEFT;
            case 2:
                return ReefSide.FRONT_RIGHT;
            case 3:
                return ReefSide.BACK;
            case 4:
                return ReefSide.BACK_LEFT;
            case 5:
                return ReefSide.BACK_RIGHT;
            default:
                return ReefSide.FRONT;
            
        }
    }
}
