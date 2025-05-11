// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

/** Add your docs here. */
public class ElevatorCommands {

  /**
   * Gives the elevator subsystem a target length
   * 
   * @param elevator - the elevator subsystem
   * @param targetLength - the length to be achieved
   * @param allowEndCondition - whether the end condition of the command is to be
   * @return - the command with the logic
   */
  public static Command elevatorToTarget(Elevator elevator, DoubleSupplier targetLength, boolean allowEndCondition) {
    if(allowEndCondition) {
      return new FunctionalCommand(
        () -> {},
        () -> {
          elevator.setTargetLength(targetLength.getAsDouble());
        },
        interrupted -> {},
        () -> Math.abs(elevator.getCurrentLength("Left") - targetLength.getAsDouble()) < ElevatorConstants.kLengthErrorAllowed,
        elevator);

    } else {
      return new FunctionalCommand(
        () -> {},
        () -> {
          elevator.setTargetLength(targetLength.getAsDouble());
        },
        interrupted -> {},
        () -> false,
        elevator);
    }
  }

  /**
   * Gives the elevator subsystem a target length
   * 
   * THIS METHOD DOES NOT USE A SUPPLIER! ONLY USE THIS IF THE TARGET IS A CONSTANT!
   * 
   * @param elevator - the elevator subsystem
   * @param targetLength - the length to be achieved
   * @param allowEndCondition - whether the end condition of the command is to be
   * @return - the command with the logic
   */
  public static Command elevatorToTarget(Elevator elevator, double targetLength, boolean allowEndCondition) {
    return elevatorToTarget(elevator, () -> targetLength, allowEndCondition);
  }

  /**
   * Sends the elevator home
   * 
   * @param elevator - the elevator subsystem
   * @return - the command with the logic
   */
  public static Command elevatorToHome(Elevator elevator, boolean allowEndCondition) {
    return elevatorToTarget(elevator, ElevatorConstants.kHomeLength, allowEndCondition);
  }

  /**
   * Hold the elevator at the current length
   * 
   * @param elevator - the elevator subsystem
   * @return - the command with the logic
   */
  public static Command elevatorHold(Elevator elevator) {
    return elevatorToTarget(elevator, () -> elevator.getCurrentLength("Left"), false);
  }
}
