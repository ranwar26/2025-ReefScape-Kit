// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.pivot.Pivot;

/** Add your docs here. */
public class PivotCommands {

  /**
   * Gives the pivot subsystem a target angle
   * 
   * @param pivot - the pivot subsystem
   * @param targetAngle - the angle to achieve
   * @param allowEndCondition - whether the end condition is used
   * @return - Command with the given logic
   */
  public static Command pivotToTarget(Pivot pivot, DoubleSupplier targetAngle, boolean allowEndCondition) {

    Command returnCommand;

    if(allowEndCondition) {
      returnCommand = new FunctionalCommand(
        () -> {},
        () -> {
          pivot.setTargetAngle(targetAngle.getAsDouble());
        },
        interrupted -> {
          pivot.setTargetAngle(pivot.getCurrentAngle());
        },
        () -> Math.abs(pivot.getCurrentAngle() - targetAngle.getAsDouble()) < PivotConstants.kAngleErrorAllowed,
        pivot).withName("pivotToTarget");

    } else {
      returnCommand = new FunctionalCommand(
        () -> {},
        () -> {
          pivot.setTargetAngle(targetAngle.getAsDouble());
        },
        interrupted -> {
          pivot.setTargetAngle(pivot.getCurrentAngle());
        },
        () -> false,
        pivot).withName("pivotToTarget");
    }

    returnCommand.setSubsystem("Pivot");

    return returnCommand;
  }

  /**
   * Gives the pivot subsystem a target angle
   * 
   * @param pivot - the pivot subsystem
   * @param targetAngle - the angle to achieve
   * @param allowEndCondition - whether the end condition is used
   * @return - Command with the given logic
   */
  public static Command pivotToTarget(Pivot pivot, double targetAngle, boolean allowEndCondition) {
    return pivotToTarget(pivot, () -> targetAngle, allowEndCondition);
  }

  /**
   * Send the pivot back to home
   * 
   * @param pivot - the pivot subsystem
   * @return - Command with the given logic
   */
  public static Command pivotToHome(Pivot pivot, boolean allowEndCondition) {
    return pivotToTarget(pivot, PivotConstants.kHomeAngle, allowEndCondition).withName("pivotToHome");
  }

  /**
   * Hold the pivot at current angle
   * 
   * @param pivot - the pivot subsystem
   * @return - Command with the given logic
   */
  public static Command pivotHold(Pivot pivot) {
    double[] target = new double[1];

    return new FunctionalCommand(
        () -> {
          target[0] = pivot.getCurrentAngle();
        },
        () -> {
          pivot.setTargetAngle(target[0]);
        },
        interrupted -> {},
        () -> false,
        pivot).withName("pivotHold");
  }
}
