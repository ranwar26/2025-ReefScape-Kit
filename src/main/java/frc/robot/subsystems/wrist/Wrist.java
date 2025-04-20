// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private ProfiledPIDController m_wristPIDControllor;

  /** Creates a new wrist. */
  public Wrist(WristIO io) {

    this.io = io;

    this.m_wristPIDControllor = new ProfiledPIDController(
        WristConstants.kP,
        WristConstants.kI,
        WristConstants.kD,
        new TrapezoidProfile.Constraints(
            WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  /**
   * Tell the pivot to move to the target length
   * 
   * @param angle - the target angle
   */
  public void setTargetAngle(double angle) {

    double deltaAngle = inputs.position - angle;

    double targetSpeed = this.m_wristPIDControllor.calculate(deltaAngle);

    this.io.setWristVolts(MathUtil.clamp(targetSpeed, -1.0, 1.0) * 12.0);
  }

  /**
   * get the current angle of the pivot
   * 
   * @return - the current angle of the side picked
   */
  public double getCurrentAngle() {
    return this.io.getCurrentAngle();
  }
}
