// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.WristConsants;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class WristCommands {

  public static Command wristToTarget(Wrist wrist, double targetAngle) {
    return Commands.run(
        () -> {
          wrist.setTargetAngle(targetAngle);
        },
        wrist);
  }

  public static Command wristToHome(Wrist wrist) {
    return wristToTarget(wrist, WristConsants.kHomeAngle);
  }
}
