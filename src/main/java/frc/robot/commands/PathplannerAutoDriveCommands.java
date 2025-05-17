// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class PathplannerAutoDriveCommands {

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
    public static Command pathFindToReef(int faceOfReef, PathConstraints constraints) {

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

        PathConstraints approachConstraints = new PathConstraints(0.5, 1.0, Math.PI / 2.0, Math.PI / 2.0);

        return new SequentialCommandGroup(
            AutoBuilder.pathfindToPose(
            targetPose.transformBy(new Transform2d(-0.75, 0.0, new Rotation2d())),
            constraints
            ),
            AutoBuilder.pathfindToPose(
            targetPose,
            approachConstraints
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

    private final class ReefPositions {

        private static final AprilTagFieldLayout aprilTagLayout = VisionConstants.aprilTagLayout;

        private static final boolean isOnBlueAlliance = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);

        private static final Transform2d positionFromTagOffset = new Transform2d(0.5, 0, new Rotation2d(Math.PI));

        //These are based on the view of the reef from the driver station of the same team
        private static final Pose2d frontReefTag =       isOnBlueAlliance ? aprilTagLayout.getTagPose(18).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(7).orElse(new Pose3d()).toPose2d();
        private static final Pose2d frontLeftReefTag =   isOnBlueAlliance ? aprilTagLayout.getTagPose(19).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(6).orElse(new Pose3d()).toPose2d();
        private static final Pose2d frontRightReefTag =  isOnBlueAlliance ? aprilTagLayout.getTagPose(17).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(8).orElse(new Pose3d()).toPose2d();
        private static final Pose2d backReefTag =        isOnBlueAlliance ? aprilTagLayout.getTagPose(21).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(10).orElse(new Pose3d()).toPose2d();
        private static final Pose2d backLeftReefTag =    isOnBlueAlliance ? aprilTagLayout.getTagPose(20).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(11).orElse(new Pose3d()).toPose2d();
        private static final Pose2d backRightReefTag =   isOnBlueAlliance ? aprilTagLayout.getTagPose(22).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(9).orElse(new Pose3d()).toPose2d();

        public static final Pose2d frontReefRobotPosition = frontReefTag.transformBy(positionFromTagOffset);
        public static final Pose2d frontLeftReefRobotPosition = frontLeftReefTag.transformBy(positionFromTagOffset);
        public static final Pose2d frontRightReefRobotPosition = frontRightReefTag.transformBy(positionFromTagOffset);
        public static final Pose2d backReefRobotPosition = backReefTag.transformBy(positionFromTagOffset);
        public static final Pose2d backLeftReefRobotPosition = backLeftReefTag.transformBy(positionFromTagOffset);
        public static final Pose2d backRightReefRobotPosition = backRightReefTag.transformBy(positionFromTagOffset);

    }

    private final class CoralPositions {

        private static final AprilTagFieldLayout aprilTagLayout = VisionConstants.aprilTagLayout;

        private static final boolean isOnBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        private static final Transform2d positionFromTagOffset = new Transform2d(0.5, 0, new Rotation2d());

        //These are based on the view of the coral station from the driver station of the same team
        private static final Pose2d leftCoralTag =  isOnBlueAlliance ? aprilTagLayout.getTagPose(13).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(1).orElse(new Pose3d()).toPose2d();
        private static final Pose2d rightCoralTag = isOnBlueAlliance ? aprilTagLayout.getTagPose(12).orElse(new Pose3d()).toPose2d() : aprilTagLayout.getTagPose(2).orElse(new Pose3d()).toPose2d();

        public static final Pose2d leftCoralRobotPosition = leftCoralTag.transformBy(positionFromTagOffset);
        public static final Pose2d rightCoralRobotPosition = rightCoralTag.transformBy(positionFromTagOffset);

    }

}
