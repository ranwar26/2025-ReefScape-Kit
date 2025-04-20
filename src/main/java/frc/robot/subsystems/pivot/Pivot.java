// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private ProfiledPIDController m_pivotPidController;

  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {

    this.io = io;

    this.m_pivotPidController = new ProfiledPIDController(
        PivotConstants.kP,
        PivotConstants.kI,
        PivotConstants.kD,
        new TrapezoidProfile.Constraints(
            PivotConstants.kMaxVelocity, PivotConstants.kMaxAcceleration));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  /**
   * Tell the pivot to move to the target length
   * 
   * @param angle - the target angle
   */
  public void setTargetAngle(double angle) {

    double deltaAngle = inputs.leftPosition - angle;

    double targetSpeed = this.m_pivotPidController.calculate(deltaAngle);

    this.io.setPivotVolts(MathUtil.clamp(targetSpeed, -1.0, 1.0) * 12.0);
  }

  /**
   * get the current angle of the pivot (Right encoder if none is given)
   * 
   * @return - the current length of the side picked
   */
  public double getCurrentAngle() {
    return this.getCurrentAngle("Right");
  }

  /**
   * get the current angle of the pivot
   * 
   * @param side - the side to grab data from
   * @return - the current angle of the side picked
   */
  public double getCurrentAngle(String side) {
    return this.io.getCurrentAngle(side);
  }
}
