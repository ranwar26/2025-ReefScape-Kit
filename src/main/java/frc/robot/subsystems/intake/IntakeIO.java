// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * An interface for the intake IO
 */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /**
   * Updates the inputs of the intake subsystem
   *
   * @param inputs the class to be updated
   */
  public default void updateInputs(IntakeIOInputs inputs) {
  }

  /**
   * Sets the motors at the given voltage
   *
   * @param volts the number of volt
   */
  public default void setIntakeVolts(double volts) {
  }

}
