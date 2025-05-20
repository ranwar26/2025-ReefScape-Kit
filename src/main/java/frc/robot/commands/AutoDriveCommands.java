// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            );
    }

    /**
     * 1 - front
     * 2 - front left
     * 3 - front right
     * 4 - back
     * 5 - back left
     * 6 - back right
     * 
     * @param faceOfReef - the target side of the reef
     * @param constraints
     * @return
     */
    public static Command pathFindToReef(Drive drive, int faceOfReef, PathConstraints constraints) {

        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        Pose2d targetPose;

        switch (faceOfReef) {
            case 1:
                targetPose = ReefPositions.frontReefRobotPosition;
                break;
            case 2:
                targetPose = ReefPositions.frontLeftReefRobotPosition;
                break;
            case 3:
                targetPose = ReefPositions.frontRightReefRobotPosition;
                break;
            case 4:
                targetPose = ReefPositions.backReefRobotPosition;
                break;
            case 5:
                targetPose = ReefPositions.backLeftReefRobotPosition;
                break;
            case 6:
                targetPose = ReefPositions.backRightReefRobotPosition;
                break;
            default:
            targetPose = new Pose2d();
            break;
        }

        return new SequentialCommandGroup(
            AutoBuilder.pathfindToPose(
            targetPose,
            constraints
            ),
            preciseMoveToPose(
                drive,
                targetPose
            )
        );
    }

    public static Command pathFindToCoralStation(boolean leftStation, PathConstraints constraints) {

        if(constraints == null) {
            constraints = new PathConstraints(DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxSpeedMetersPerSec, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularSpeed);
        }

        Pose2d targetPose  = leftStation ? CoralPositions.leftCoralRobotPosition : CoralPositions.rightCoralRobotPosition;

        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0
        );
    }


    public static PIDController preciseMovePIDController = new PIDController(1.0, 0, 0);

    /**
     * Only run when near the target, as it does not use pathfinding. 
     * 
     * @param drive
     * @param pose
     * @return
     */
    public static Command preciseMoveToPose(Drive drive, Pose2d targetPose) {

        Transform2d[] deltaPose = new Transform2d[1];

        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    Pose2d currentRobotPose = drive.getPose();

                    deltaPose[0] = currentRobotPose.minus(targetPose).times(-1);
                }
            ),
            DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 1.0,
            () -> preciseMovePIDController.calculate(deltaPose[0].getX()),
            () -> preciseMovePIDController.calculate(deltaPose[0].getY()),
            () -> targetPose.getRotation().getCos(),
            () -> targetPose.getRotation().getSin()
            ).withTimeout(0.02)
        ).repeatedly().withTimeout(1.0);

    }

}
