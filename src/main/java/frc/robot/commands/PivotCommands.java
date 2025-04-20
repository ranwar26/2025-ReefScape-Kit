// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  public static Command pivotToTarget(Pivot pivot, double targetAngle, boolean allowEndCondition) {
    if(allowEndCondition) {
      return new FunctionalCommand(
        () -> {},
        () -> {
          pivot.setTargetAngle(targetAngle);
        },
        interrupted -> {},
        () -> Math.abs(pivot.getCurrentAngle("Left") - targetAngle) < PivotConstants.kAngleErrorAllowed,
        pivot);

    } else {
      return new FunctionalCommand(
        () -> {},
        () -> {
          pivot.setTargetAngle(targetAngle);
        },
        interrupted -> {},
        () -> false,
        pivot);
    }
  }

  /**
   * Send the pivot back to home
   * 
   * @param pivot - the pivot subsystem
   * @return - Command with the given logic
   */
  public static Command pivotToHome(Pivot pivot) {
    return pivotToTarget(pivot, PivotConstants.kHomeAngle, true);
  }
}
