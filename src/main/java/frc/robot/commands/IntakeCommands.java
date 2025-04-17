// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeCommands {

  public static Command intakeRun(Intake intake, DoubleSupplier speedPercentage) {

    return Commands.run(
        () -> {
          intake.setOutput(speedPercentage.getAsDouble());
        },
        intake);
  }
}
