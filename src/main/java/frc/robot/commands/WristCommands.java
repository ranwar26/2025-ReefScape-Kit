// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class WristCommands {

  /**
   * Gives the wrist subsystem a target angle
   * 
   * @param wrist - the wrist subsystem
   * @param targetAngle - the angle to be achieved
   * @param allowEndCondition
   * @return - the command with the given logic
   */
  public static Command wristToTarget(Wrist wrist, DoubleSupplier targetAngle, boolean allowEndCondition) {

    Command returnCommand;

    if(allowEndCondition) {
      returnCommand = new FunctionalCommand(
        () -> {},
        () -> {
          wrist.setTargetAngle(targetAngle.getAsDouble());
        },
        interrupted -> {
          wrist.setTargetAngle(wrist.getCurrentAngle());
        },
        () -> Math.abs(wrist.getCurrentAngle() - targetAngle.getAsDouble()) < WristConstants.kAngleErrorAllowed,
        wrist).withName("wristToTarget");

    } else {
      returnCommand = new FunctionalCommand(
        () -> {},
        () -> {
          wrist.setTargetAngle(targetAngle.getAsDouble());
        },
        interrupted -> {
          wrist.setTargetAngle(wrist.getCurrentAngle());
        },
        () -> false,
        wrist).withName("wristToTarget");
    }

    returnCommand.setSubsystem("Wrist");

    return returnCommand;
  }

  /**
   * Gives the wrist subsystem a target angle
   * 
   * @param wrist - the wrist subsystem
   * @param targetAngle - the angle to be achieved
   * @param allowEndCondition
   * @return - the command with the given logic
   */
  public static Command wristToTarget(Wrist wrist, double targetAngle, boolean allowEndCondition) {
    return wristToTarget(wrist, () -> targetAngle, allowEndCondition);
  }

  /**
   * Sends the wrist back to home
   * 
   * @param wrist - the wrist subsystem
   * @return - the command with the given logic
   */
  public static Command wristToHome(Wrist wrist, boolean allowEndCondition) {
    Command returnCommand = wristToTarget(wrist, WristConstants.kHomeAngle, allowEndCondition).withName("wristToHome");
    returnCommand.setSubsystem("Wrist");
    return returnCommand;
  }

  /**
   * Hold the wrist at current angle
   * 
   * @param wrist - the wrist subsystem
   * @return - the command with the given logic
   */
  public static Command wristHold(Wrist wrist) {
    double[] target = new double[1];

    return new FunctionalCommand(
        () -> {
          target[0] = wrist.getCurrentAngle();
        },
        () -> {
          wrist.setTargetAngle(target[0]);
        },
        interrupted -> {},
        () -> false,
        wrist).withName("wristHold");
  }
}
