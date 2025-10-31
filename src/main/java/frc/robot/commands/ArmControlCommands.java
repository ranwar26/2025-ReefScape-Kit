// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/**
 * A class containing arm control command for moving up, down, and holding at a position.
 */
public class ArmControlCommands {

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

    // Convert the enum into target positions
    double[] subsystemTargets = getSubsystemPositions(target);
    double pivotTarget = subsystemTargets[0];
    double elevatorTarget = subsystemTargets[1];
    double wristTarget = subsystemTargets[2];

    // Get which subsystems to control
    List<ArmSystem> activeSystems = controlledSystemSetup(controlledSystems);

    // Set up the command structure
    SequentialCommandGroup returnCommand = new SequentialCommandGroup();
    ParallelDeadlineGroup targetStateOne = new ParallelDeadlineGroup(Commands.waitSeconds(0.0));
    ParallelDeadlineGroup targetStateTwo = new ParallelDeadlineGroup(Commands.waitSeconds(0.0));
    ParallelDeadlineGroup targetStateThree = new ParallelDeadlineGroup(Commands.waitSeconds(0.0));

    // Reset PID controllers
    returnCommand.addCommands(Commands.runOnce(() -> {
      pivot.resetPID();
      elevator.resetPID();
      wrist.resetPID();
    }));

    //Add cdd the parts to the return command
    returnCommand.addCommands(targetStateOne, targetStateTwo, targetStateThree);

    if(activeSystems.contains(ArmSystem.PIVOT)) { // Pivot related commands, wait on pivot for stage 1
      targetStateOne.setDeadline(PivotCommands.pivotToTarget(pivot, pivotTarget, true));
      targetStateTwo.addCommands(PivotCommands.pivotToTarget(pivot, pivotTarget, false));
      targetStateThree.addCommands(PivotCommands.pivotToTarget(pivot, pivotTarget, false));
    }

    if(activeSystems.contains(ArmSystem.ELEVATOR)) { // Elevator related commands, wait on elevator for stage 2
      targetStateOne.addCommands(ElevatorCommands.elevatorHold(elevator));
      targetStateTwo.setDeadline(ElevatorCommands.elevatorToTarget(elevator, elevatorTarget, true));
      targetStateThree.addCommands(ElevatorCommands.elevatorToTarget(elevator, elevatorTarget, false));
    }

    if(activeSystems.contains(ArmSystem.WRIST)) { // Wrist related commands, wait on wrist for stage 3
      targetStateOne.addCommands(WristCommands.wristHold(wrist));
      targetStateTwo.addCommands(WristCommands.wristHold(wrist));
      targetStateThree.setDeadline(WristCommands.wristToTarget(wrist, wristTarget, true));
    }

    return returnCommand.withName("ArmUpCommand");
  }

  /**
   * Returns a command that brings the arm to it's home state from a given start state.
   *
   * @param pivot The pivot subsystem
   * @param elevator The Elevator subsystem
   * @param wrist The Wrist subsystem
   * @param fromState The starting or where the arm is coming from. If unknown use null.
   * @return A command with the given logic
   */
  public static Command armDownCommand(Pivot pivot, Elevator elevator, Wrist wrist, ArmPosition fromState) {

    // If the arm state could be in a non-normal state (end of auto), use unknown command.
    if(fromState == null) {
      return armDownUnknownCommand(pivot, elevator, wrist);
    }

    // Convert the enum to target positions
    double[] subsystemTargets = getSubsystemPositions(fromState);
    double pivotTarget = subsystemTargets[0];
    double elevatorTarget = subsystemTargets[1];

    // Only the return command is need, due to this command requiring all subsystems
    SequentialCommandGroup returnCommand = new SequentialCommandGroup();

    // Reset PID controllers
    returnCommand.addCommands(Commands.runOnce(() -> {
      pivot.resetPID();
      elevator.resetPID();
      wrist.resetPID();
    }));

    // Stage 1 waits on wrist
    returnCommand.addCommands(new ParallelDeadlineGroup(
      WristCommands.wristToHome(wrist, true),
      PivotCommands.pivotToTarget(pivot, pivotTarget, false),
      ElevatorCommands.elevatorToTarget(elevator, elevatorTarget, false)
    ));

    // Stage 2 waits on elevator
    returnCommand.addCommands(new ParallelDeadlineGroup(
      ElevatorCommands.elevatorToHome(elevator, true),
      PivotCommands.pivotToTarget(pivot, pivotTarget, false),
      WristCommands.wristToHome(wrist, false)
    ));

    // Stage 3 waits on pivot
    returnCommand.addCommands(new ParallelDeadlineGroup(
      PivotCommands.pivotToHome(pivot, true),
      ElevatorCommands.elevatorToHome(elevator, false),
      WristCommands.wristToHome(wrist, false)
    ));

    return returnCommand.withName("ArmDownCommand");
  }

  /**
   * Returns a command that moves the arm to home position without relying on holding at a know position.
   * Tends to be useful for where many different arm position could be posable (End of auto).
   *
   * @param pivot the pivot subsystem
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @return the command with the given logic
   */
  private static Command armDownUnknownCommand(Pivot pivot, Elevator elevator, Wrist wrist) {

    // The return command
    SequentialCommandGroup returnCommand = new SequentialCommandGroup();

    // Resets the PID controller
    returnCommand.addCommands(Commands.runOnce(() -> {
      pivot.resetPID();
      elevator.resetPID();
      wrist.resetPID();
    }));

    // Stage 1 waits on wrist
    returnCommand.addCommands(new ParallelDeadlineGroup(
      WristCommands.wristToHome(wrist, true),
      PivotCommands.pivotHold(pivot),
      ElevatorCommands.elevatorHold(elevator)
    ));

    // Stage 2 waits on elevator
    returnCommand.addCommands(new ParallelDeadlineGroup(
      ElevatorCommands.elevatorToHome(elevator, false),
      PivotCommands.pivotHold(pivot)
    ));

    // Stage 3 waits on pivot
    returnCommand.addCommands(
      PivotCommands.pivotToHome(pivot, false)
    );

    return returnCommand.withName("ArmDownCommand");

  }

  /**
   * Returns a command that forces subsystems to hold their current position (not their target!), until
   * some other command calls them.
   *
   * @param pivot The pivot subsystem
   * @param elevator The elevator subsystem
   * @param wrist The wrist subsystem
   * @param holdPosition the target position to hold at
   * @param controlledSystems which system to have hold
   * @return A command with the given logic
   */
  public static Command armHoldAtCommand(Pivot pivot, Elevator elevator, Wrist wrist, ArmPosition holdPosition, ArmSystem... controlledSystems) {

    // Converts the enum to target positions
    double[] subsystemHold = getSubsystemPositions(holdPosition);
    double pivotHold = subsystemHold[0];
    double elevatorHold = subsystemHold[1];
    double wristHold = subsystemHold[2];

    // Gets which subsystems to control
    List<ArmSystem> activeSystems = controlledSystemSetup(controlledSystems);

    // The command to return
    ParallelCommandGroup returnCommand = new ParallelCommandGroup();

    // Reset the PID controllers
    returnCommand.addCommands(Commands.runOnce(() -> {
      pivot.resetPID();
      elevator.resetPID();
      wrist.resetPID();
    }));

    if(activeSystems.contains(ArmSystem.PIVOT)) { // Holds the pivot
      returnCommand.addCommands(PivotCommands.pivotToTarget(pivot, pivotHold, false));
    }
    if(activeSystems.contains(ArmSystem.ELEVATOR)) { // Holds the elevator
      returnCommand.addCommands(ElevatorCommands.elevatorToTarget(elevator, elevatorHold, false));
    }
    if(activeSystems.contains(ArmSystem.WRIST)) { // holds the wrist
      returnCommand.addCommands(WristCommands.wristToTarget(wrist, wristHold, false));
    }

    return returnCommand;
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

    // May want to just have a method in the Enum class with this method.
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
      case PAST_STAGE2:
        returnArray = new double[]{PivotConstants.kLevel3Angle, ElevatorConstants.kPastStage2, WristConstants.kPastStage2};
        break;
      case CORAL_STATION:
        returnArray = new double[]{PivotConstants.kCoralStationAngle, ElevatorConstants.kCoralStationLength, WristConstants.kCoralStationAngle};
        break;
      case LOWER_ALGAE_REMOVE:
        returnArray = new double[]{PivotConstants.kLowerAlgaeRemove, ElevatorConstants.kLevel2Length, WristConstants.kLowerAlgaeRemove};
        break;
      case HIGHER_ALGAE_REMOVE:
        returnArray = new double[]{PivotConstants.kHigherAlgaeRemove, ElevatorConstants.kLevel3Length, WristConstants.kHigherAlgaeRemove};
        break;
      case HOME:
        returnArray = new double[]{PivotConstants.kHomeAngle, ElevatorConstants.kHomeLength, WristConstants.kCoralStationAngle};
        break;
      case CAGE:
        returnArray = new double[]{PivotConstants.kCageStowAngle, ElevatorConstants.kHomeLength, WristConstants.kCoralStationAngle};
        break;
      case STAND_BY:
        returnArray = new double[]{PivotConstants.kLevel3Angle, ElevatorConstants.kHomeLength, WristConstants.kLevel1Angle};
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

    /** Elevator starts stage 2 */
    PAST_STAGE2,

    /** Coral station */
    CORAL_STATION,

    /** Lower Algae */
    LOWER_ALGAE_REMOVE,

    /** Higher Algae */
    HIGHER_ALGAE_REMOVE,

    /** The Home State */
    HOME,

    /** The cage grab position */
    CAGE,

    /** A hold position for DynamicAuto */
    STAND_BY
  }
}
