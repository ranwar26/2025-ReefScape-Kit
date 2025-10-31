// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class with the IO for the Wrist
 */
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
   * Tell the wrist to move to the target angle
   *
   * @param angle the target angle
   */
  public void setTargetAngle(double angle) {
    this.io.setTargetAngle(angle);
  }

  /**
   * Gets the target angle of the wrist
   *
   * @return the target angle
   */
  public double getTargetAngle() {
    return this.inputs.targetPosition;
  }

  /**
   * Gets the current angle of the wrist
   *
   * @return the current angle
   */
  public double getCurrentAngle() {
    return this.inputs.currentPosition;
  }

  /**
   * Resets the PID controller
   */
  public void resetPID() {
    this.io.resetPID();
  }
}
