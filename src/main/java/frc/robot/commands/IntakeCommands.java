// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

/**
 * A class containing intake commands
 */
public class IntakeCommands {

  /**
   * Runs the intake a speed percentage
   *
   * @param intake          the intake subsystem
   * @param speedPercentage the percentage the intake should run at
   * @return - the command with the logic
   */
  public static Command intakeRun(Intake intake, DoubleSupplier speedPercentage) {

    return Commands.run(
        () -> {
          intake.setIntakePercentage(speedPercentage.getAsDouble());
        },
        intake)
        .withName("intakeRun");
  }
}
