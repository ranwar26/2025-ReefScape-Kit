// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

/** Add your docs here. */
public final class MotorConfigs {

  public final class DriveConfig {

    public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

    static {
      driveConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
          .voltageCompensation(12.0);
      driveConfig
          .encoder
          .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
          .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
          .uvwMeasurementPeriod(10)
          .uvwAverageDepth(2);
      driveConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              DriveConstants.driveKp, 0.0,
              DriveConstants.driveKd, 0.0);
      driveConfig
          .signals
          .primaryEncoderPositionAlwaysOn(true)
          .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
          .primaryEncoderVelocityAlwaysOn(true)
          .primaryEncoderVelocityPeriodMs(20)
          .appliedOutputPeriodMs(20)
          .busVoltagePeriodMs(20)
          .outputCurrentPeriodMs(20);

      turnConfig
          .inverted(DriveConstants.turnInverted)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
          .voltageCompensation(12.0);
      turnConfig
          .absoluteEncoder
          .inverted(DriveConstants.turnEncoderInverted)
          .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
          .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
          .averageDepth(2);
      turnConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(
              DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
          .pidf(DriveConstants.turnKp, 0.0, DriveConstants.turnKd, 0.0);
      turnConfig
          .signals
          .absoluteEncoderPositionAlwaysOn(true)
          .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
          .absoluteEncoderVelocityAlwaysOn(true)
          .absoluteEncoderVelocityPeriodMs(20)
          .appliedOutputPeriodMs(20)
          .busVoltagePeriodMs(20)
          .outputCurrentPeriodMs(20);
    }
  }

  public static final class PivotConfig {
    public static final SparkMaxConfig leftPivotConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightPivotConfig = new SparkMaxConfig();

    static {
      leftPivotConfig.inverted(false);
      rightPivotConfig.inverted(true);

      // Changes all inputs and outputs from motor rotations to pivot angle in radians
      double positionConversionFactorRelative =
          (2.0 * Math.PI) / PivotConstants.motorToPivotAngleRatio;
      leftPivotConfig.encoder.positionConversionFactor(positionConversionFactorRelative);
      rightPivotConfig.encoder.positionConversionFactor(positionConversionFactorRelative);

      rightPivotConfig.absoluteEncoder.inverted(true);

      leftPivotConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(PivotConstants.kMaxCurrentLimit)
          .voltageCompensation(PivotConstants.kMaxVoltage);
      rightPivotConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(PivotConstants.kMaxCurrentLimit)
          .voltageCompensation(PivotConstants.kMaxVoltage);
    }
  }

  public static final class ElevatorConfig {
    public static final SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();

    static {
      leftElevatorConfig.inverted(false);
      rightElevatorConfig.inverted(true);

      // Turns inputs and outputs from motor rotation into the extension
      // 0.02425 is r (2.0 * Math.PI * 0.02425) = 0.15236724 meters, 0.16269 seems to
      // be a better conversion?
      double positionConversionFactor = (0.16269) / ElevatorConstants.motorToDrumRatio;

      leftElevatorConfig.encoder.positionConversionFactor(positionConversionFactor);
      rightElevatorConfig.encoder.positionConversionFactor(positionConversionFactor);

      leftElevatorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ElevatorConstants.kMaxCurrentLimit)
          .voltageCompensation(ElevatorConstants.kMaxVoltage);
      rightElevatorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ElevatorConstants.kMaxCurrentLimit)
          .voltageCompensation(ElevatorConstants.kMaxVoltage);
    }
  }

  public static final class WristConfig {
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();

    static {
      wristConfig.inverted(true);

      // Changes all inputs and outputs from motor rotations to pivot angle in radians
      double positionConversionFactorRelative = (2 * Math.PI) / WristConstants.motorToWheelRatio;

      wristConfig.encoder.positionConversionFactor(positionConversionFactorRelative);

      wristConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(WristConstants.kMaxCurrentLimit)
          .voltageCompensation(WristConstants.kMaxVoltage);
    }
  }

  public static final class IntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      intakeConfig.inverted(false);

      intakeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(IntakeConstants.kMaxCurrentLimit)
          .voltageCompensation(IntakeConstants.kMaxVoltage);
    }
  }
}
