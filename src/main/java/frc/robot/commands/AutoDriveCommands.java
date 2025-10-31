// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants.CoralPositions;
import frc.robot.FieldConstants.ReefPositions;
import frc.robot.subsystems.drive.Drive;

/**
 * A class containing Auto drive command using PathPlanner's on-the-fly pathfinding
 */
public class AutoDriveCommands {

    /**
     * Builds a command that moves the robot to the target pose, given the constrains.
     *
     * @param targetPose The goal pose of the robot at the end of the command
     * @param constraints The constraints of the path
     * @param targetEndVelocity The goal velocity at the end of the command
     * @return A command with the given logic
     */
    public static Command pathFindToPose(Supplier<Pose2d> targetPose, PathConstraints constraints, double targetEndVelocity) {

        // If no constraints are given, use the DriveConstants max.
        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        return AutoBuilder.pathfindToPose(
            targetPose.get(),
            constraints,
            targetEndVelocity
        ).withName("pathFindToPose");
    }

    /**
     * Builds a command that move the robot to the selected side of the reef, given the constraints. With the
     * possibly to add a precise move command, which lines up more after Pathplanner's command ends.
     *
     * @param drive The drive subsystem
     * @param faceOfReef The side of the reef to move towards
     * @param constraints The constraints of the path
     * @param withPreciseMove Whether to chain the command into a precise move command
     * @return A command with the given logic
     */
    public static Command pathFindToReef(Drive drive, ReefSide faceOfReef, PathConstraints constraints, boolean withPreciseMove) {

        // If no constraints are given, use the DriveConstants max.
        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        // Convert the enum into a target pose
        Pose2d targetPose;

        // May use a method in the Enum class
        switch (faceOfReef) {
            case FRONT:
                targetPose = ReefPositions.frontReefRobotPosition;
                break;
            case FRONT_LEFT:
                targetPose = ReefPositions.frontLeftReefRobotPosition;
                break;
            case FRONT_RIGHT:
                targetPose = ReefPositions.frontRightReefRobotPosition;
                break;
            case BACK:
                targetPose = ReefPositions.backReefRobotPosition;
                break;
            case BACK_LEFT:
                targetPose = ReefPositions.backLeftReefRobotPosition;
                break;
            case BACK_RIGHT:
                targetPose = ReefPositions.backRightReefRobotPosition;
                break;
            default:
                targetPose = new Pose2d(); // Shouldn't be callable
                break;
        }

        if(withPreciseMove) {
            // Appends the precise move command
            return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
                ),
                preciseMoveToPose(
                    drive,
                    targetPose
                )
            ).withName("pathFindToReef");
        }

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
            ).withName("pathFindToReef");
    }

    /**
     * Builds a command that move the robot to the selected coral station, given the constraints. With the
     * possibly to add a precise move command, which lines up more after Pathplanner's command ends.
     *
     * @param drive the drive subsystem
     * @param leftStation whether to move to the left stations or the right station (True = left)
     * @param constraints the constrains of the path
     * @param withPreciseMove Whether to chain the command into a precise move command
     * @return A command with the given logic
     */
    public static Command pathFindToCoralStation(Drive drive, boolean leftStation, PathConstraints constraints, boolean withPreciseMove) {

        // If no constraints are given, use the DriveConstants max.
        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        // Gets the target pose
        Pose2d targetPose  = leftStation ? CoralPositions.leftCoralRobotPosition : CoralPositions.rightCoralRobotPosition;

        if(withPreciseMove) {
            // Appends the precise move command
            return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
                ),
                preciseMoveToPose(
                    drive,
                    targetPose
                )
            ).withName("pathFindToCoralStation");
        }
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0
        ).withName("pathFindToCoralStation");
    }


    private static PIDController preciseXMovePIDController = new PIDController(1.0, 0.1, 0.0);
    private static PIDController preciseYMovePIDController = new PIDController(1.0, 0.1, 0.0);

    /**
     * Uses PID controllers to move the robot into the near perfect pose. No rotation. Doesn't
     * use pathfinding, so only use when nothing exists in the way.
     *
     * @param drive the drive subsystem
     * @param targetPose the target pose
     * @return A command with the given logic
     */
    public static Command preciseMoveToPose(Drive drive, Pose2d targetPose) {

        double[] deltaValues = new double[2];

        // Auto uses stricter pose to reduce auto error
        Pose2d maxErrorPose = DriverStation.isAutonomous() ? AutoDriveConstants.maxErrorPoseAuto : AutoDriveConstants.maxErrorPoseTeleop;

        // A supplier for if the robot is within error
        BooleanSupplier isWithinError = () ->
            Math.abs(drive.getPose().minus(targetPose).getX()) < maxErrorPose.getX() &&
            Math.abs(drive.getPose().minus(targetPose).getY()) < maxErrorPose.getY();

        // Standard debouncer
        Debouncer debouncer = new Debouncer(0.25, DebounceType.kBoth);

        return new SequentialCommandGroup(
            Commands.runOnce(
                () -> {
                    // Calculate the delta values on X and Y
                    Pose2d currentRobotPose = drive.getPose();

                    deltaValues[0] = currentRobotPose.getX() - targetPose.getX();
                    deltaValues[1] = currentRobotPose.getY() - targetPose.getY();
                }
            ),
            // Command the the drive
            DriveCommands.driveAtAngle(
                drive,
                () -> 1.0,
                () -> preciseXMovePIDController.calculate(deltaValues[0]),
                () -> preciseYMovePIDController.calculate(deltaValues[1]),
                () -> targetPose.getRotation().getCos(),
                () -> targetPose.getRotation().getSin()
            )
            .withTimeout(0.02)
        )
        .repeatedly()
        .until(() -> debouncer.calculate(isWithinError.getAsBoolean()))
        .beforeStarting(() -> {
            preciseXMovePIDController.reset();
            preciseYMovePIDController.reset();
        })
        .andThen(Commands.runOnce(
            () -> drive.stopWithX()
        )).withName("preciseMoveToPose");

    }

    /**
     * The six sides of the reef
     */
    public static enum ReefSide {

        /** Faces the driver stations */
        FRONT,

        /** Faces the driver stations */
        FRONT_LEFT,

        /** Faces the driver stations */
        FRONT_RIGHT,

        /** Faces away from the driver stations */
        BACK,

        /** Faces away from the driver stations */
        BACK_LEFT,

        /** Faces away from the driver stations */
        BACK_RIGHT
    }

}
