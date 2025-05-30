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


    //Part for the current arm
    private static LoggedMechanism2d mechanismCurrent = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPartCurrent = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPartCurrent = new LoggedMechanismLigament2d("gripper", 0.388, 0);
    private static LoggedMechanismLigament2d intakePartCurrent = new LoggedMechanismLigament2d("intake", 0.05, 0);

    //Part for the target arm
    private static LoggedMechanism2d mechanismTarget = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPartTarget = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPartTarget = new LoggedMechanismLigament2d("gripper", 0.388, 0);
    private static LoggedMechanismLigament2d intakePartTarget = new LoggedMechanismLigament2d("intake", 0.05, 0);

    static {
        mechanismCurrent.getRoot("root", 2.5-0.3, 0.2).append(elevatorPartCurrent).append(gripperPartCurrent).append(intakePartCurrent);

        elevatorPartCurrent.setColor(new Color8Bit(0, 255, 0));
        gripperPartCurrent.setColor(new Color8Bit(0, 255, 0));
        intakePartCurrent.setColor(new Color8Bit(0, 255, 0));

        elevatorPartCurrent.setLineWeight(4.0);
        gripperPartCurrent.setLineWeight(4.0);
        intakePartCurrent.setLineWeight(4.0);

        mechanismTarget.getRoot("root", 2.5-0.3, 0.2).append(elevatorPartTarget).append(gripperPartTarget).append(intakePartTarget);

        elevatorPartTarget.setColor(new Color8Bit(255, 0, 0));
        gripperPartTarget.setColor(new Color8Bit(255, 0, 0));
        intakePartTarget.setColor(new Color8Bit(255, 0, 0));

        elevatorPartTarget.setLineWeight(3.9);
        gripperPartTarget.setLineWeight(3.9);
        intakePartTarget.setLineWeight(3.9);
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
    public static Command mechanismRunCurrent(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

    return Commands.run(
        () -> {
            elevatorPartCurrent.setAngle(Math.toDegrees(pivot.getCurrentAngle()));
            elevatorPartCurrent.setLength(elevator.getCurrentLength());

            gripperPartCurrent.setAngle(Math.toDegrees(-wrist.getCurrentAngle()) + 90.0);

            intakePartCurrent.setAngle(Math.toDegrees(intake.getCurrentAngle()));

            Logger.recordOutput("Arm System/Current", mechanismCurrent);

        }).withName("mechanismRunCurrent");
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
    public static Command mechanismRunTarget(Pivot pivot, Elevator elevator, Wrist wrist, Intake intake) {

        return Commands.run(
            () -> {
                elevatorPartTarget.setAngle(Math.toDegrees(pivot.getTargetAngle()));
                elevatorPartTarget.setLength(elevator.getTargetLength());
    
                gripperPartTarget.setAngle(Math.toDegrees(-wrist.getTargetAngle()) + 90.0);
    
                intakePartTarget.setAngle(Math.toDegrees(intake.getCurrentAngle()));
    
                Logger.recordOutput("Arm System/Target", mechanismTarget);
    
            }).withName("mechanismRunTarget");
        }
}
