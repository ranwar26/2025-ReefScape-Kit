// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A class for controller commands
 */
public class ControllerCommands {

  /**
   * Sets the controllers rumble for a time, then stopping it
   *
   * @param controller  the controller
   * @param rumbleValue the strength of the rumble (0.0-1.0)
   * @param time        the amount of time (Sec) the command should run
   * @return A command with the given logic
   */
  public static Command setRumble(CommandXboxController controller, double rumbleValue, double time) {
    double[] endTime = new double[1];

    return new FunctionalCommand(
        () -> {
          endTime[0] = DriverStation.getMatchTime() - time;
        },
        () -> {
          controller.setRumble(RumbleType.kBothRumble, rumbleValue);
        },
        interrupted -> {
          controller.setRumble(RumbleType.kBothRumble, 0.0);
        },
        () -> DriverStation.getMatchTime() < endTime[0]).withName("setRumble");
  }
}
