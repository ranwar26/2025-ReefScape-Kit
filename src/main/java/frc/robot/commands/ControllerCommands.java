// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class ControllerCommands {

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
            () -> DriverStation.getMatchTime() < endTime[0]
            ).withName("setRumble");
    }
}
