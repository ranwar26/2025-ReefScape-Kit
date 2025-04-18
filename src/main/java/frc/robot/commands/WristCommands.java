// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class WristCommands {

  public static Command wristToTarget(Wrist wrist, double targetAngle, boolean allowEndCondition) {
    if(allowEndCondition) {
      return new FunctionalCommand(
        () -> {},
        () -> {
          wrist.setTargetAngle(targetAngle);
        },
        interrupted -> {},
        () -> Math.abs(wrist.getCurrentAngle() - targetAngle) < WristConstants.kAngleErrorAllowed,
        wrist);

    } else {
      return new FunctionalCommand(
        () -> {},
        () -> {
          wrist.setTargetAngle(targetAngle);
        },
        interrupted -> {},
        () -> false,
        wrist);
    }
  }

  public static Command wristToHome(Wrist wrist) {
    return wristToTarget(wrist, WristConstants.kHomeAngle, false);
  }
}
