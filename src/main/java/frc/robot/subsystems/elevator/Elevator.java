// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {

    this.io = io;

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /**
   * Tell the elevator to move to the target length
   * 
   * @param length - the target length
   */
  public void setTargetLength(double length) {
    this.io.setTargetLength(length);
  }

  /**
   * get the current Length of the elevator (Right encoder if none is given)
   * 
   * @return - the current length of the side picked
   */
  public double getCurrentLength() {
    return this.io.getCurrentLength();
  }

}
