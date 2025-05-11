// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class ArmControlCommandGroups {

  /**
   * Sequential commands for the arm to score on level 2 of the reef
   * 
   * @param pivot - the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command Level2UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, true), // Command group waits on this

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kLevel2Angle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false)
        )
    );
  }

  /**
   * Sequential commands for the arm to score on level 3 of the reef
   * 
   * @param pivot - the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command Level3UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, true), // Command group waits on this

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kLevel3Angle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false)
        )
    );
  }

  /**
   * Sequential commands for the arm to score on level 4 of the reef
   * 
   * @param pivot - the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command Level4UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, true), // Command group waits on this

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kLevel4Angle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false)
        )
    );
  }

  /**
   * Sequential commands for the arm to grab from the coral station
   * 
   * @param pivot - the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command coralStationUpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kCoralStationAngle, true), // Command group waits on this

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kCoralStationLength, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kCoralStationAngle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kCoralStationAngle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kCoralStationLength, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kCoralStationAngle, false)
        )
    );
  }

  /**
   * Sequential commands for the arm to retract safely
   * 
   * @param pivot - the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command retractCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        new ParallelDeadlineGroup(
          WristCommands.wristToHome(wrist), // Command group waits on this
          ElevatorCommands.elevatorHold(elevator),
          PivotCommands.pivotHold(pivot)
        ),

        new ParallelDeadlineGroup(
          ElevatorCommands.elevatorToHome(elevator), // Command group waits on this
          PivotCommands.pivotHold(pivot)
        ),

        PivotCommands.pivotToHome(pivot) // Command group waits on this
    );
  }
}
