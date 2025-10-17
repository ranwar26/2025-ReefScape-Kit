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

package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 0.01;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 2 * Math.PI;
  private static final double ANGLE_MAX_ACCELERATION = 2 * Math.PI;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Turning PID controller
  private static final ProfiledPIDController angleController = new ProfiledPIDController(
      ANGLE_KP,
      0.0,
      ANGLE_KD,
      new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  private DriveCommands() {
  }

  /**
   * Turns the position of a joystick into a x, y velocity
   *
   * @param x - x position of the joystick
   * @param y - y position of the joystick
   * @return x, y velocity
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Turns the position of a joystick into a a rotational speed
   * 
   * @param x - x position of the joystick
   * @param y - y position of the joystick
   * @param rawGyroAngle - Current gyro angle in degrees
   * @return - the rotational speed in radians
   */
  private static Rotation2d getAngularSpeedFromJoysticks(double x, double y, double rawGyroAngle) {

    //Checks if any rotation input is being given
    if (Math.abs(x) < OIConstants.kDriveDeadband && Math.abs(y) < OIConstants.kDriveDeadband) {
      return new Rotation2d(0.0);
    }

    //put angle between 0 and 359 degrees
    double currentAngle = Math.abs(rawGyroAngle % 360);
    if (rawGyroAngle < 0) {
      currentAngle = 360 - currentAngle;
    }

    currentAngle *= -1;

    double targetAngle = Math.atan2(x, y);
    targetAngle = (Math.toDegrees(targetAngle) - 90) % 360;

    // This should digitally "notch" the joystick to the 6 sides of the reef.
    targetAngle = (Math.round(targetAngle / 60.0) * 60.0) % 360.0;

    double theta = Math.abs(targetAngle - currentAngle) % 360;
    double shorterTheta = theta > 180 ? 360 - theta : theta;

    int sign = -1;
    double angleDelta = currentAngle - targetAngle;

    if (angleDelta >= 0 && angleDelta <= 180) {
      sign = 1;
    } else if (angleDelta <= -180 && angleDelta >= -360) {
      sign = 1;
    }

    double correctiveAngle = shorterTheta * sign;
    double rotSpeed = -angleController.calculate(correctiveAngle);

    return new Rotation2d(rotSpeed);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control.
   * 
   * @param drive - the drive subsystem
   * @param throttleSupplier - the throttle of the robot speed
   * @param xSupplier - x speed of the robot
   * @param ySupplier - y speed of the robot
   * @param xAngleSupplier - x component of the angle
   * @param yAngleSupplier - y component of the angle
   * @return - the command with the logic of this method
   */
  public static Command driveAtAngle(
      Drive drive,
      DoubleSupplier throttleSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier xAngleSupplier,
      DoubleSupplier yAngleSupplier) {

    // Construct command
    Command returnCommand = Commands.run(
        () -> {

          double throttle = throttleSupplier.getAsDouble();
          double xSpeed = xSupplier.getAsDouble();
          double ySpeed = ySupplier.getAsDouble();

          // modifies the inputs for the goal of the following:
          // (1). if throttle is 0, then the movement joystick has a max of quarter speed
          // (2). As the throttle is increased, the speed moves from quarter speed
          // (joystick is being full pushed) to kMaxSpeed
          xSpeed *= 0.25;
          ySpeed *= 0.25;
          throttle *= 3.0;
          throttle += 1.0;

          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSpeed, ySpeed);
          linearVelocity = new Translation2d(xSpeed, ySpeed);

          // Calculate angular speed
          Rotation2d rotationVelocity = getAngularSpeedFromJoysticks(xAngleSupplier.getAsDouble(),
              yAngleSupplier.getAsDouble(), drive.getRotation().getDegrees());

          // Convert the commanded speeds into the correct units for the drivetrain
          double xSpeedDelivered = throttle * linearVelocity.getX() * DriveConstants.maxSpeedMetersPerSec;
          double ySpeedDelivered = throttle * linearVelocity.getY() * DriveConstants.maxSpeedMetersPerSec;
          double rotDelivered = rotationVelocity.getRadians() * DriveConstants.maxAngularSpeed;

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeedDelivered,
              ySpeedDelivered,
              rotDelivered,
              drive.getRotation());

          drive.runVelocity(speeds);
        },
        drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians())).withName("driveAtAngle");

        returnCommand.setSubsystem("Drive");

        return returnCommand;
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                })).withName("feedforwardCharacterization");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    }))).withName("wheelRadiusCharacterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
