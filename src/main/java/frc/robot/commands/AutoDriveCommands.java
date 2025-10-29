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

/** Add your docs here. */
public class AutoDriveCommands {

    public static Command pathFindToPose(Supplier<Pose2d> targetPose, PathConstraints constraints, double targetEndVelocity) {

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
     * 
     * @param faceOfReef - the target side of the reef
     * @param constraints
     * @return
     */
    public static Command pathFindToReef(Drive drive, ReefSide faceOfReef, PathConstraints constraints, boolean withPreciseMove) {

        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        Pose2d targetPose;

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
                targetPose = new Pose2d();
                break;
        }

        if(withPreciseMove) {
            return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(
                targetPose,
                constraints
                ),
                preciseMoveToPose(
                    drive,
                    targetPose
                )
            ).withName("pathFindToReef");
        }
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints
            ).withName("pathFindToReef");
    }

    public static Command pathFindToCoralStation(Drive drive, boolean leftStation, PathConstraints constraints, boolean withPreciseMove) {

        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        Pose2d targetPose  = leftStation ? CoralPositions.leftCoralRobotPosition : CoralPositions.rightCoralRobotPosition;

        if(withPreciseMove) {
            return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(
                targetPose,
                constraints
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
     * Only run when near the target, as it does not use pathfinding. 
     * 
     * @param drive
     * @param pose
     * @return
     */
    public static Command preciseMoveToPose(Drive drive, Pose2d targetPose) {

        double[] deltaValues = new double[2];

        Pose2d maxErrorPose = DriverStation.isAutonomous() ? AutoDriveConstants.maxErrorPoseAuto : AutoDriveConstants.maxErrorPoseTeleop;

        BooleanSupplier isWithinError = () ->
            Math.abs(drive.getPose().minus(targetPose).getX()) < maxErrorPose.getX() &&
            Math.abs(drive.getPose().minus(targetPose).getY()) < maxErrorPose.getY();

        Debouncer debouncer = new Debouncer(0.1, DebounceType.kBoth);

        return new SequentialCommandGroup(
            Commands.runOnce(
                () -> {
                    Pose2d currentRobotPose = drive.getPose();

                    deltaValues[0] = currentRobotPose.getX() - targetPose.getX();
                    deltaValues[1] = currentRobotPose.getY() - targetPose.getY();
                }
            ),
            DriveCommands.driveAtAngle(
            drive,
            () -> 1.0,
            () -> preciseXMovePIDController.calculate(deltaValues[0]),
            () -> preciseYMovePIDController.calculate(deltaValues[1]),
            () -> targetPose.getRotation().getCos(),
            () -> targetPose.getRotation().getSin()
            ).withTimeout(0.02)
        ).repeatedly().until(() -> debouncer.calculate(isWithinError.getAsBoolean()))
        .beforeStarting(() -> {
            preciseXMovePIDController.reset();
            preciseYMovePIDController.reset();
        })
        .andThen(Commands.runOnce(
            () -> drive.stopWithX()
        )).withName("preciseMoveToPose");

    }

    public static enum ReefSide {

        FRONT,

        FRONT_LEFT,

        FRONT_RIGHT,

        BACK,

        BACK_LEFT,

        BACK_RIGHT
    }

}
