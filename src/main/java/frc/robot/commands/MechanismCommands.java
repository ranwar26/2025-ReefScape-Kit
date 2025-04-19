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

    private static LoggedMechanism2d mechanismLeft = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPartLeft = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPartLeft = new LoggedMechanismLigament2d("gripper", 0.3, 0);
    private static LoggedMechanismLigament2d intakePartLeft = new LoggedMechanismLigament2d("intake", 0.05, 0);

    private static LoggedMechanism2d mechanismRight = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPartRight = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPartRight = new LoggedMechanismLigament2d("gripper", 0.3, 0);
    private static LoggedMechanismLigament2d intakePartRight = new LoggedMechanismLigament2d("intake", 0.05, 0);

    static {
        mechanismLeft.getRoot("root", 2.5-0.3, 0.2).append(elevatorPartLeft).append(gripperPartLeft).append(intakePartLeft);

        elevatorPartLeft.setColor(new Color8Bit(255, 0, 0));
        gripperPartLeft.setColor(new Color8Bit(255, 0, 0));
        intakePartLeft.setColor(new Color8Bit(255, 0, 0));

        elevatorPartLeft.setLineWeight(4.0);
        gripperPartLeft.setLineWeight(4.0);
        intakePartLeft.setLineWeight(4.0);

        mechanismRight.getRoot("root", 2.5-0.3, 0.2).append(elevatorPartRight).append(gripperPartRight).append(intakePartRight);

        elevatorPartRight.setColor(new Color8Bit(0, 0, 255));
        gripperPartRight.setColor(new Color8Bit(0, 0, 255));
        intakePartRight.setColor(new Color8Bit(0, 0, 255));

        elevatorPartRight.setLineWeight(4.0);
        gripperPartRight.setLineWeight(4.0);
        intakePartRight.setLineWeight(4.0);
    }

    public static Command mechanismRun(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    return Commands.run(
        () -> {
            elevatorPartLeft.setAngle(Math.toDegrees(pivot.getCurrentAngle("Left")));
            elevatorPartLeft.setLength(0.7 + elevator.getCurrentLength("Left"));

            gripperPartLeft.setAngle(Math.toDegrees(-wrist.getCurrentAngle()) + 90.0);

            intakePartLeft.setAngle(Math.toDegrees(intake.getCurrentAngle()));

            Logger.recordOutput("Arm System/Left", mechanismLeft);

            elevatorPartRight.setAngle(Math.toDegrees(-pivot.getCurrentAngle("Right")));
            elevatorPartRight.setLength(0.7 + -elevator.getCurrentLength("Right"));

            gripperPartRight.setAngle(Math.toDegrees(-wrist.getCurrentAngle()) + 90.0);

            intakePartRight.setAngle(Math.toDegrees(intake.getCurrentAngle()));

            Logger.recordOutput("Arm System/Right", mechanismRight);

            
        },
        new Subsystem[] {}).ignoringDisable(true);
    }
}
