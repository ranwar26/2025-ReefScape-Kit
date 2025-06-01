// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class NEW_ArmControlCommands {

  /**
   * Returns a command to move the arm to the target position. The subsystems move in the order: Pivot, Elevator, Wrist.
   * The optional 5th+ parameter takes ArmSystem members as the systems to be controlled. Important: Selecting a subsystem
   * without the lower subsystems before it will: not move the lower subsystems, but still move the selected subsystem,
   * this can cause unexpect damage. So, when selecting a subsystem; include lower subsystems, unless you know what you are doing.
   * 
   * @param pivot The pivot subsystem
   * @param elevator The elevator subsystem
   * @param wrist The Wrist subsystem
   * @param target The target state of the arm
   * @param controlledSystems Which arm systems to be controlled
   * @return A command with the given logic
   */
  public static Command armUpCommand(Pivot pivot, Elevator elevator, Wrist wrist, ArmPosition target, ArmSystem... controlledSystems) {

    double[] subsystemTargets = getSubsystemPositions(target);

    double pivotTarget = subsystemTargets[0];
    double elevatorTarget = subsystemTargets[1];
    double wristTarget = subsystemTargets[2];

    List<ArmSystem> activeSystems = controlledSystemSetup(controlledSystems);

    SequentialCommandGroup returnCommand = new SequentialCommandGroup();
    ParallelDeadlineGroup targetStateOne = new ParallelDeadlineGroup(Commands.waitSeconds(0.0));
    ParallelDeadlineGroup targetStateTwo = new ParallelDeadlineGroup(Commands.waitSeconds(0.0));
    ParallelDeadlineGroup targetStateThree = new ParallelDeadlineGroup(Commands.waitSeconds(0.0));

    returnCommand.addCommands(targetStateOne, targetStateTwo, targetStateThree);

    if(activeSystems.contains(ArmSystem.PIVOT)) {
      targetStateOne.setDeadline(PivotCommands.pivotToTarget(pivot, pivotTarget, true));
      targetStateTwo.addCommands(PivotCommands.pivotToTarget(pivot, pivotTarget, false));
      targetStateThree.addCommands(PivotCommands.pivotToTarget(pivot, pivotTarget, false));
    }

    if(activeSystems.contains(ArmSystem.ELEVATOR)) {
      targetStateTwo.setDeadline(ElevatorCommands.elevatorToTarget(elevator, elevatorTarget, true));
      targetStateThree.addCommands(ElevatorCommands.elevatorToTarget(elevator, elevatorTarget, false));
    }

    if(activeSystems.contains(ArmSystem.WRIST)) {
      targetStateThree.setDeadline(WristCommands.wristToTarget(wrist, wristTarget, true));
    }

    return returnCommand.withName("ArmUpCommand");
  }

  public static Command armDownCommand(Pivot pivot, Elevator elevator, Wrist wrist, ArmPosition fromState) {
    
    double[] subsystemTargets = getSubsystemPositions(fromState);

    double pivotTarget = subsystemTargets[0];
    double elevatorTarget = subsystemTargets[1];
    double wristTarget = subsystemTargets[2];


    SequentialCommandGroup returnCommand = new SequentialCommandGroup();
    
    returnCommand.addCommands(new ParallelDeadlineGroup(
      WristCommands.wristToHome(wrist, true),
      PivotCommands.pivotToTarget(pivot, pivotTarget, false),
      ElevatorCommands.elevatorToTarget(elevator, elevatorTarget, false)
    ));

    returnCommand.addCommands(new ParallelDeadlineGroup(
      ElevatorCommands.elevatorToHome(elevator, false),
      PivotCommands.pivotToTarget(pivot, pivotTarget, false)
    ));

    returnCommand.addCommands(
      PivotCommands.pivotToHome(pivot, false)
    );

    return returnCommand.withName("ArmDownCommand");
  }

  /**
   * This method turns an ArmTarget into a three part array for each subsystem's position.
   * The array has the following values in the given position: 0-pivot 1-elevator 2-wrist
   * 
   * @param position the position
   * @return An array of the position values
   */
  private static double[] getSubsystemPositions(ArmPosition position) {

    double[] returnArray = null;

    switch (position) {
      case LEVEL2:
        returnArray = new double[]{PivotConstants.kLevel2Angle, ElevatorConstants.kLevel2Length, WristConstants.kLevel2Angle};
        break;
      case LEVEL3:
        returnArray = new double[]{PivotConstants.kLevel3Angle, ElevatorConstants.kLevel3Length, WristConstants.kLevel3Angle};
        break;
      case LEVEL4:
        returnArray = new double[]{PivotConstants.kLevel4Angle, ElevatorConstants.kLevel4Length, WristConstants.kLevel4Angle};
        break;
      case CORAL_STATION:
        returnArray = new double[]{PivotConstants.kCoralStationAngle, ElevatorConstants.kCoralStationLength, WristConstants.kCoralStationAngle};
        break;
      
    }

    return returnArray;
  }

  /**
   * Converts the inputted Array into a list for easy searching. Also if the array contains the
   * ArmSystem.ALL then the method adds all other ArmSystems.
   * 
   * @param controlledSystems Array with the controlled arm systems.
   * @return the proper list of ArmSystems to be controlled
   */
  private static List<ArmSystem> controlledSystemSetup(ArmSystem[] controlledSystems) {

    List<ArmSystem> activeSystems = new ArrayList<>(Arrays.asList(controlledSystems));

    if(activeSystems.contains(ArmSystem.ALL)) {
      activeSystems.clear();
      activeSystems.add(ArmSystem.PIVOT);
      activeSystems.add(ArmSystem.ELEVATOR);
      activeSystems.add(ArmSystem.WRIST);
    }

    return activeSystems;
  }

  /**
   * An enum for the arm systems. Used as inputs to the ArmControlCommands methods.
   */
  public static enum ArmSystem {

    /** Used for all Arm Systems */
    ALL,

    /** The Pivot subsystem */
    PIVOT,

    /** The Elevator subsystem */
    ELEVATOR,

    /** The Wrist subsystem */
    WRIST

  }

  /**
   * An enum for the arm targets.
   */
  public static enum ArmPosition {
    
    /** Reef level 2 */
    LEVEL2,

    /** Reef level 3 */
    LEVEL3,

    /** Reef level 4 */
    LEVEL4,

    /** Coral station */
    CORAL_STATION
  }
}
