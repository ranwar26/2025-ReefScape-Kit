// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A class that hold the IO of the pivot */
public class Pivot extends SubsystemBase {

  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {

    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  /**
   * Tell the pivot to move to the target length
   *
   * @param angle the target angle
   */
  public void setTargetAngle(double angle) {
    this.io.setTargetAngle(angle);
  }

  public double getTargetAngle() {
    return this.inputs.targetPosition;
  }

  /**
   * Gets the current angle of the pivot
   *
   * @return the current angle of the pivot
   */
  public double getCurrentAngle() {
    return this.inputs.currentPosition;
  }

  /** Resets the PID controller */
  public void resetPID() {
    this.io.resetPID();
  }
}
