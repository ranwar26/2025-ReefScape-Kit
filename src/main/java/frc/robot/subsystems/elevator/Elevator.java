// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ProfiledPIDController m_elevatorPidController;

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {

    this.io = io;

    this.m_elevatorPidController =
      new ProfiledPIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(
          ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration
    ));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setTargetLength(double length) {

    double deltaLength = inputs.leftPosition - length;

    double targetSpeed = this.m_elevatorPidController.calculate(deltaLength);

    this.io.setElevatorVolts(MathUtil.clamp(targetSpeed, -1.0, 1.0) * 12.0);
  }

  public double getCurrentLength() {
    return this.io.getCurrentLength();
  }
}
