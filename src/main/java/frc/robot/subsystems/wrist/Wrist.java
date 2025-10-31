// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Creates a new wrist. */
  public Wrist(WristIO io) {

    this.io = io;

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
    this.io.setTargetAngle(angle);
  }

  public double getTargetAngle() {
    return this.inputs.targetPosition;
  }

  /**
   * get the current angle of the pivot
   * 
   * @return - the current angle
   */
  public double getCurrentAngle() {
    return this.inputs.position;
  }

  /**
   * resets the PID controller
   */
  public void resetPID() {
    this.io.resetPID();
  }
}
