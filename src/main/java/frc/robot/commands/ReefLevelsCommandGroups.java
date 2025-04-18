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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class ReefLevelsCommandGroups {

  public static Command Level2UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, true),

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, true),
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kLevel2Angle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel2Length, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel2Angle, false)
        )
    );
  }

  public static Command Level3UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, true),

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, true),
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kLevel3Angle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel3Length, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel3Angle, false)
        )
    );
  }

  public static Command Level4UpCommandGroup(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    return new SequentialCommandGroup(

        PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, true),

        new ParallelDeadlineGroup(
              ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, true),
            PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false)
        ),

        new ParallelCommandGroup(
          WristCommands.wristToTarget(wrist, WristConstants.kLevel4Angle, false),
          ElevatorCommands.elevatorToTarget(elevator, ElevatorConstants.kLevel4Length, false),
          PivotCommands.pivotToTarget(pivot, PivotConstants.kLevel4Angle, false)
        )
    );
  }

}
