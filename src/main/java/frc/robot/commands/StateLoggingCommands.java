// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;

/** Add your docs here. */
public class StateLoggingCommands {

    // Part for the current arm
    private static LoggedMechanism2d mechanismCurrent = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPartCurrent = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPartCurrent = new LoggedMechanismLigament2d("gripper", 0.388, 0);
    private static LoggedMechanismLigament2d intakePartCurrent = new LoggedMechanismLigament2d("intake", 0.05, 0);

    // Part for the target arm
    private static LoggedMechanism2d mechanismTarget = new LoggedMechanism2d(5, 5.0);
    private static LoggedMechanismLigament2d elevatorPartTarget = new LoggedMechanismLigament2d("elevator", 0.0, 0);
    private static LoggedMechanismLigament2d gripperPartTarget = new LoggedMechanismLigament2d("gripper", 0.388, 0);
    private static LoggedMechanismLigament2d intakePartTarget = new LoggedMechanismLigament2d("intake", 0.05, 0);

    static {
        mechanismCurrent.getRoot("root", 2.5 - 0.3, 0.2).append(elevatorPartCurrent).append(gripperPartCurrent)
                .append(intakePartCurrent);

        elevatorPartCurrent.setColor(new Color8Bit(0, 255, 0));
        gripperPartCurrent.setColor(new Color8Bit(0, 255, 0));
        intakePartCurrent.setColor(new Color8Bit(0, 255, 0));

        elevatorPartCurrent.setLineWeight(4.0);
        gripperPartCurrent.setLineWeight(4.0);
        intakePartCurrent.setLineWeight(4.0);

        mechanismTarget.getRoot("root", 2.5 - 0.3, 0.2).append(elevatorPartTarget).append(gripperPartTarget)
                .append(intakePartTarget);

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
     * @param pivot    - the pivot subsystem
     * @param elevator - the elevator subsystem
     * @param wrist    - the wrist subsystem
     * @param intake   - the intake subsystem
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
     * @param pivot    - the pivot subsystem
     * @param elevator - the elevator subsystem
     * @param wrist    - the wrist subsystem
     * @param intake   - the intake subsystem
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

    // ########## Commands Logging ##########
    private static final List<Command> activeCommands = new ArrayList<>();

    private static final List<String> allCommandNames = new ArrayList<>();
    private static final List<String> driveCommandNames = new ArrayList<>();
    private static final List<String> pivotCommandNames = new ArrayList<>();
    private static final List<String> elevatorCommandNames = new ArrayList<>();
    private static final List<String> wristCommandNames = new ArrayList<>();
    private static final List<String> intakeCommandNames = new ArrayList<>();

    private static final List<List<String>> allSubsystemCommandNames = new ArrayList<>(
            Arrays.asList(driveCommandNames, pivotCommandNames, elevatorCommandNames, wristCommandNames,
                    intakeCommandNames));

    public static Command logCommands() {

        CommandScheduler.getInstance().onCommandInitialize(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                activeCommands.add(command);
            }
        });
        CommandScheduler.getInstance().onCommandFinish(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                activeCommands.remove(command);
            }
        });
        CommandScheduler.getInstance().onCommandInterrupt(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                activeCommands.remove(command);
            }
        });

        return Commands.run(() -> {
            allCommandNames.clear();
            driveCommandNames.clear();
            pivotCommandNames.clear();
            elevatorCommandNames.clear();
            wristCommandNames.clear();
            intakeCommandNames.clear();

            for (Command command : activeCommands.toArray(new Command[0])) {

                allCommandNames.add(command.getName());

                switch (command.getSubsystem()) {
                    case "Drive":
                        driveCommandNames.add(command.getName());
                        break;
                    case "Pivot":
                        pivotCommandNames.add(command.getName());
                        break;
                    case "Elevator":
                        elevatorCommandNames.add(command.getName());
                        break;
                    case "Wrist":
                        wristCommandNames.add(command.getName());
                        break;
                    case "Intake":
                        intakeCommandNames.add(command.getName());
                        break;
                }
            }

            for (List<String> subsystemCommandNames : allSubsystemCommandNames) {
                if (subsystemCommandNames.size() > 1) {
                    Elastic.sendNotification(new Notification(Notification.NotificationLevel.WARNING, "Double Command",
                            "Currently " + subsystemCommandNames.size() + " commands are running on a subsystem, with names: " + subsystemCommandNames.toString()));

                    System.out.println( "Warning: " +
                            "Currently " + subsystemCommandNames.size() + " commands are running on a subsystem, with names: " + subsystemCommandNames.toString());

                }
            }

            Logger.recordOutput("Active Commands/All", allCommandNames.toArray(new String[0]));
            Logger.recordOutput("Active Commands/Subsystem/DriveCommands", driveCommandNames.toArray(new String[0]));
            Logger.recordOutput("Active Commands/Subsystem/PivotCommands", pivotCommandNames.toArray(new String[0]));
            Logger.recordOutput("Active Commands/Subsystem/ElevatorCommands", elevatorCommandNames.toArray(new String[0]));
            Logger.recordOutput("Active Commands/Subsystem/WristCommands", wristCommandNames.toArray(new String[0]));
            Logger.recordOutput("Active Commands/Subsystem/IntakeCommands", intakeCommandNames.toArray(new String[0]));
        }).ignoringDisable(true).withName("CommandLoggerCommand");
    }

}
