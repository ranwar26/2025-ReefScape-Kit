// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

/** Add your docs here. */
public class ElevatorCommands {

  public static Command elevatorToTarget(Elevator elevator, double targetLength) {
    return Commands.run(
        () -> {
          elevator.setTargetLength(targetLength);
        },
        elevator);
  }

  public static Command wristToHome(Elevator elevator) {
    return elevatorToTarget(elevator, ElevatorConstants.kHomeLength);
  }
}
