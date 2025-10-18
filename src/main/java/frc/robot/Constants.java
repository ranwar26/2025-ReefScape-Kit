// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double maxAngularSpeed = 2 * Math.PI;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(29.0);
    public static final double wheelBase = Units.inchesToMeters(29.0);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions.
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2.0);

    // SPARK MAX CAN IDs
    public static final int frontLeftDriveCanId = 7;
    public static final int backLeftDriveCanId = 5;
    public static final int frontRightDriveCanId = 9;
    public static final int backRightDriveCanId = 3;

    public static final int frontLeftTurnCanId = 6;
    public static final int backLeftTurnCanId = 4;
    public static final int frontRightTurnCanId = 8;
    public static final int backRightTurnCanId = 2;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 40;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction =
        (45.0 * 22.0) / (13.0 * 15.0); // MAXSwerve with 13 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.4;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.5;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 50.219;
    public static final double robotMOI = 4.859;
    public static final double wheelCOF = 1.190;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec - 0.8,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
}

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(-0.5, 0.20, 0.0, new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.5, -0.20, 0.0, new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}

  public final class PivotConstants {

    public static final int kLeftMotorID = 10;
    public static final int kRightMotorID = 11;

    public static final DCMotor motorGearbox = DCMotor.getNEO(1);

    public static final double motorToPivotAngleRatio = 213.33;

    public static final int kMaxCurrentLimit = 20;
    public static final double kMaxVoltage = 12;

    public static final double kRealP = 1.2;
    public static final double kRealI = 0.0;
    public static final double kRealD = 0.0;
    public static final double kSimP = 5.0;
    public static final double kSimI = 0.2;
    public static final double kSimD = 0.4;

    // Target angle in degrees converted to radians
    public static final double kHomeAngle = Math.toRadians(40.0);

    public static final double kLevel1Angle = Math.toRadians(0.0);
    public static final double kLevel2Angle = Math.toRadians(60.0);
    public static final double kLevel3Angle = Math.toRadians(72.0);
    public static final double kLevel4Angle = Math.toRadians(80.0);
    public static final double kCoralStationAngle = Math.toRadians(74.0);

    public static final double kUpperAlgaeRemove = Math.toRadians(67.0);
    public static final double kLowerAlgaeRemove = Math.toRadians(59.0);

    // The error limit before a command will end.
    public static final double kAngleErrorAllowed = Math.toRadians(10.0);
  }

  public final class ElevatorConstants {

    public static final int kLeftMotorID = 12;
    public static final int kRightMotorID = 13;

    public static final DCMotor motorGearbox = DCMotor.getNEO(1);

    public static final double motorToDrumRatio = 7.2;

    public static final int kMaxCurrentLimit = 60;
    public static final double kMaxVoltage = 12;

    public static final double kRealP = 2.7;
    public static final double kRealI = 0.0;
    public static final double kRealD = 0.0;
    public static final double kSimP = 0.9;
    public static final double kSimI = 1.2;
    public static final double kSimD = 0.2;

    // Target length in meters
    public static final double kHomeLength = 0.659;

    public static final double kLevel1Length = 0.659;
    public static final double kLevel2Length = 0.849;
    public static final double kLevel3Length = 1.169;
    public static final double kLevel4Length = 1.959;
    public static final double kCoralStationLength = 0.689;

    // The number of meter the elevator has to move to active the second stage.
    public static final double kSecondStageTrip = 0.75;

    // The error limit before a command will end.
    public static final double kLengthErrorAllowed = 0.1;
  }

  public final class WristConstants {

    public static final int kMotorID = 14;

    public static final DCMotor motorGearbox = DCMotor.getNEO(1);

    public static final double motorToWheelRatio = 48.0;

    public static final int kMaxCurrentLimit = 60;
    public static final double kMaxVoltage = 12;

    public static final double kRealP = 0.4;
    public static final double kRealI = 0.0;
    public static final double kRealD = 0.0;
    public static final double kSimP = 0.9;
    public static final double kSimI = 0.0;
    public static final double kSimD = 0.3;

    // Target angle in degrees converted to radians
    public static final double kHomeAngle = Math.toRadians(0.0);

    public static final double kLevel1Angle = Math.toRadians(90.0);
    public static final double kLevel2Angle = Math.toRadians(168.0);
    public static final double kLevel3Angle = Math.toRadians(180.0);
    public static final double kLevel4Angle = Math.toRadians(192.0);
    public static final double kCoralStationAngle = Math.toRadians(26.0);

    public static final double kUpperAlgaeRemove = Math.toRadians(157.0);
    public static final double kLowerAlgaeRemove = Math.toRadians(155.0);

    // The error limit before a command will end.
    public static final double kAngleErrorAllowed = Math.toRadians(1.0);
  }

  public final class IntakeConstants {

    public static final int kMotorID = 15;

    public static final DCMotor motorGearbox = DCMotor.getNEO(1);

    public static final double motorToWheelRatio = 3.0;

    public static final int kMaxCurrentLimit = 40;
    public static final double kMaxVoltage = 12;
  }

  public final class OIConstants {

    public static final double kDriveDeadband = 0.05;
  }

}
