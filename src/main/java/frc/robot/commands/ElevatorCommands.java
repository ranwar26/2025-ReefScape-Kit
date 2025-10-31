// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

/**
 * A class containing elevator commands
 */
public class ElevatorCommands {

  /**
   * Gives the elevator subsystem a target length
   *
   * @param elevator          the elevator subsystem
   * @param targetLength      the length to be achieved
   * @param allowEndCondition whether the end condition of the command is to be
   * @return the command with the logic
   */
  public static Command elevatorToTarget(Elevator elevator, DoubleSupplier targetLength, boolean allowEndCondition) {

    Command returnCommand = Commands.runEnd(
        () -> {
          elevator.setTargetLength(targetLength.getAsDouble());
        },
        () -> {
          elevator.setTargetLength(elevator.getCurrentLength());
        },
        elevator)
        .until(allowEndCondition
            ? () -> Math
                .abs(elevator.getCurrentLength() - targetLength.getAsDouble()) < ElevatorConstants.kLengthErrorAllowed
            : () -> false)
        .withName("elevatorToTarget");

    return returnCommand;
  }

  /**
   * Gives the elevator subsystem a target length
   *
   * @param elevator          the elevator subsystem
   * @param targetLength      the length to be achieved
   * @param allowEndCondition whether the end condition of the command is to be
   * @return the command with the logic
   */
  public static Command elevatorToTarget(Elevator elevator, double targetLength, boolean allowEndCondition) {
    return elevatorToTarget(elevator, () -> targetLength, allowEndCondition);
  }

  /**
   * Sends the elevator home
   *
   * @param elevator the elevator subsystem
   * @return the command with the logic
   */
  public static Command elevatorToHome(Elevator elevator, boolean allowEndCondition) {
    return elevatorToTarget(elevator, ElevatorConstants.kHomeLength, allowEndCondition).withName("elevatorToHome");
  }

  /**
   * Hold the elevator at the current length
   *
   * @param elevator the elevator subsystem
   * @return the command with the logic
   */
  public static Command elevatorHold(Elevator elevator) {
    return elevatorToTarget(elevator, elevator.getCurrentLength(), false).withName("elevatorHold");
  }
}
