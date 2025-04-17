// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public final class MotorConfigs {

  public final class DriveConfig {

    public final static SparkMaxConfig driveConfig = new SparkMaxConfig();
    public final static SparkMaxConfig turnConfig = new SparkMaxConfig();

    static {
      driveConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
          .voltageCompensation(12.0);
      driveConfig.encoder
          .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
          .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
          .uvwMeasurementPeriod(10)
          .uvwAverageDepth(2);
      driveConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              DriveConstants.driveKp, 0.0,
              DriveConstants.driveKd, 0.0);
      driveConfig.signals
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
      turnConfig.absoluteEncoder
          .inverted(DriveConstants.turnEncoderInverted)
          .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
          .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
          .averageDepth(2);
      turnConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
          .pidf(DriveConstants.turnKp, 0.0, DriveConstants.turnKd, 0.0);
      turnConfig.signals
          .absoluteEncoderPositionAlwaysOn(true)
          .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
          .absoluteEncoderVelocityAlwaysOn(true)
          .absoluteEncoderVelocityPeriodMs(20)
          .appliedOutputPeriodMs(20)
          .busVoltagePeriodMs(20)
          .outputCurrentPeriodMs(20);
    }

  }
}
