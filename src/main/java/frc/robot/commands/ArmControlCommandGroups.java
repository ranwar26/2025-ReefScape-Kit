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
   * @param pivot             - the pivot subsystem
   * @param elevator          - the elevator subsystem
   * @param wrist             - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command level2UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        new ParallelDeadlineGroup(
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, true), // Command group waits on this
            ElevatorCommands.elevatorToHome(elevator, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            WristCommands.wristToTarget(wrist, WristConstants.kLevel2Angle, true), // Command group waits on this
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, false),
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false)))
        .withName("level2UpCommandGroup");
  }

  public static Command holdAtLevel2CommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {
    return new ParallelCommandGroup(
        WristCommands.wristToTarget(wrist, WristConstants.kLevel2Angle, false),
        ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, false),
        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false)
    ).withName("holdAtLevel2CommandGroup");
  }

  /**
   * Sequential commands for the arm to score on level 3 of the reef
   * 
   * @param pivot             - the pivot subsystem
   * @param elevator          - the elevator subsystem
   * @param wrist             - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command level3UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        new ParallelDeadlineGroup(
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, true), // Command group waits on this
            ElevatorCommands.elevatorToHome(elevator, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            WristCommands.wristToTarget(wrist, WristConstants.kLevel3Angle, true), // Command group waits on this
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, false),
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false)))
        .withName("level3UpCommandGroup");

  }

  public static Command holdAtLevel3CommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {
    return new ParallelCommandGroup(
        WristCommands.wristToTarget(wrist, WristConstants.kLevel3Angle, false),
        ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, false),
        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false)
    ).withName("holdAtLevel3CommandGroup");
  }

  /**
   * Sequential commands for the arm to score on level 4 of the reef
   * 
   * @param pivot             - the pivot subsystem
   * @param elevator          - the elevator subsystem
   * @param wrist             - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command level4UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        new ParallelDeadlineGroup(
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, true), // Command group waits on this
            ElevatorCommands.elevatorToHome(elevator, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, true), // Command group waits on this
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            WristCommands.wristToTarget(wrist, WristConstants.kLevel4Angle, true), // Command group waits on this
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, false),
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false)))
        .withName("level4UpCommandGroup");

  }

  public static Command holdAtLevel4CommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {
    return new ParallelCommandGroup(
        WristCommands.wristToTarget(wrist, WristConstants.kLevel4Angle, false),
        ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, false),
        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false)
    ).withName("holdAtLevel4CommandGroup");
  }

  /**
   * Sequential commands for the arm to grab from the coral station
   * 
   * @param pivot             - the pivot subsystem
   * @param elevator          - the elevator subsystem
   * @param wrist             - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command coralStationUpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        new ParallelDeadlineGroup(
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kCoralStationLength, true), // Command group waits on this
            PivotCommands.pivotToHome(pivot, false),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            WristCommands.wristToTarget(wrist, WristConstants.kCoralStationAngle, true), // Command group waits on this
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kCoralStationLength, false),
            PivotCommands.pivotToHome(pivot, false)),

        new ParallelDeadlineGroup(
            PivotCommands.pivotToTarget(pivot, PivotConstants.kCoralStationAngle, true), // Command group waits on this
            WristCommands.wristToTarget(wrist, WristConstants.kCoralStationAngle, false),
            ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kCoralStationLength, false))

    ).withName("coralStationUpCommandGroup");

  }

  public static Command holdAtCoralStationCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {
    return new ParallelCommandGroup(
        WristCommands.wristToTarget(wrist, WristConstants.kCoralStationAngle, false),
        ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kCoralStationLength, false),
        PivotCommands.pivotToTarget(pivot, PivotConstants.kCoralStationAngle, false)
    ).withName("holdAtCoralStationCommandGroup");
  }

  /**
   * Sequential commands for the arm to retract safely
   * 
   * @param pivot    - the pivot subsystem
   * @param elevator - the elevator subsystem
   * @param wrist    - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command retractCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new SequentialCommandGroup(

        new ParallelDeadlineGroup(
            WristCommands.wristToHome(wrist, true), // Command group waits on this
            ElevatorCommands.elevatorHold(elevator),
            PivotCommands.pivotHold(pivot)),

        new ParallelDeadlineGroup(
            ElevatorCommands.elevatorToHome(elevator, true), // Command group waits on this
            PivotCommands.pivotHold(pivot),
            WristCommands.wristToHome(wrist, false)),

        new ParallelDeadlineGroup(
            PivotCommands.pivotToHome(pivot, true), // Command group waits on this
            WristCommands.wristToHome(wrist, false),
            ElevatorCommands.elevatorToHome(elevator, false)))
        .withName("retractCommandGroup");

  }

  /**
   * Parallel commands to hold the arm at current state.
   * 
   * @param pivot    - the pivot subsystem
   * @param elevator - the elevator subsystem
   * @param wrist    - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command holdCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist) {

    return new ParallelCommandGroup(
        PivotCommands.pivotHold(pivot),
        ElevatorCommands.elevatorHold(elevator),
        WristCommands.wristHold(wrist)).withName("holdCommandGroup");
  }

  /**
   * Command for the arm to enter home state WITHOUT SAFETY!
   * 
   * @param pivot    - the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist    - the wrist subsystem
   * @return sequentialCommandGroup - the command with the given logic
   */
  public static Command homeCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist, boolean allowEndCondition) {

    return new ParallelCommandGroup(
        WristCommands.wristToHome(wrist, allowEndCondition),
        ElevatorCommands.elevatorToHome(elevator, allowEndCondition),
        PivotCommands.pivotToHome(pivot, allowEndCondition)).withName("homeCommandGroup");
  }
}
