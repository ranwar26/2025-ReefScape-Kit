// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.DoubleSupplier;

/** A class containing wrist commands */
public class WristCommands {

  /**
   * Gives the wrist subsystem a target angle
   *
   * @param wrist the wrist subsystem
   * @param targetAngle the angle to be achieved
   * @param allowEndCondition whether the end condition is used
   * @return the command with the given logic
   */
  public static Command wristToTarget(
      Wrist wrist, DoubleSupplier targetAngle, boolean allowEndCondition) {

    Command returnCommand =
        Commands.runEnd(
                () -> {
                  wrist.setTargetAngle(targetAngle.getAsDouble());
                },
                () -> {
                  wrist.setTargetAngle(wrist.getCurrentAngle());
                },
                wrist)
            .until(
                allowEndCondition
                    ? () ->
                        Math.abs(wrist.getCurrentAngle() - targetAngle.getAsDouble())
                            < WristConstants.kAngleErrorAllowed
                    : () -> false)
            .withName("wristToTarget");

    return returnCommand;
  }

  /**
   * Gives the wrist subsystem a target angle
   *
   * @param wrist the wrist subsystem
   * @param targetAngle the angle to be achieved
   * @param allowEndCondition whether the end condition is used
   * @return the command with the given logic
   */
  public static Command wristToTarget(Wrist wrist, double targetAngle, boolean allowEndCondition) {
    return wristToTarget(wrist, () -> targetAngle, allowEndCondition);
  }

  /**
   * Sends the wrist back to home
   *
   * @param wrist the wrist subsystem
   * @return the command with the given logic
   */
  public static Command wristToHome(Wrist wrist, boolean allowEndCondition) {
    return wristToTarget(wrist, WristConstants.kHomeAngle, allowEndCondition)
        .withName("wristToHome");
  }

  /**
   * Hold the wrist at current angle
   *
   * @param wrist the wrist subsystem
   * @return the command with the given logic
   */
  public static Command wristHold(Wrist wrist) {
    return wristToTarget(wrist, wrist.getCurrentAngle(), false).withName("wristHold");
  }
}
