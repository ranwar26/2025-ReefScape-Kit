// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

/** Add your docs here. */
public class MechanismCommands {


    //Part for the right side
    private static LoggedMechanism2d mechanism = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPart = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPart = new LoggedMechanismLigament2d("gripper", 0.3, 0);
    private static LoggedMechanismLigament2d intakePart = new LoggedMechanismLigament2d("intake", 0.05, 0);

    static {
        mechanism.getRoot("root", 2.5-0.3, 0.2).append(elevatorPart).append(gripperPart).append(intakePart);

        elevatorPart.setColor(new Color8Bit(0, 0, 255));
        gripperPart.setColor(new Color8Bit(0, 0, 255));
        intakePart.setColor(new Color8Bit(0, 0, 255));

        elevatorPart.setLineWeight(4.0);
        gripperPart.setLineWeight(4.0);
        intakePart.setLineWeight(4.0);
    }

    /**
     * Updates the arm mechanism's position and rotation
     * 
     * @param pivot - the pivot subsystem
     * @param elevator - the elevator subsystem
     * @param wrist - the wrist subsystem
     * @param intake - the intake subsystem
     * @return the command with the Logic
     */
    public static Command mechanismRun(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    return Commands.run(
        () -> {
            elevatorPart.setAngle(Math.toDegrees(-pivot.getCurrentAngle("Right")));
            elevatorPart.setLength(0.7 + -elevator.getCurrentLength());

            gripperPart.setAngle(Math.toDegrees(-wrist.getCurrentAngle()) + 90.0);

            intakePart.setAngle(Math.toDegrees(intake.getCurrentAngle()));

            Logger.recordOutput("Arm System/Right", mechanism);

        },
        new Subsystem[] {});
    }
}
