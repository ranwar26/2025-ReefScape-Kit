// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /**
   * Updates the inputs of the wrist subsystem
   * 
   * @param inputs - the class to be updated
   */
  public default void updateInputs(WristIOInputs inputs) {
  }

  /**
   * Sets the motors at the given voltage
   * 
   * @param volts - the number of volt
   */
  public default void setWristVolts(double volts) {
  }

  /**
   * get the current angle of the wrist
   * 
   * @return - the current angle of the side picked
   */
  public default double getCurrentAngle() {
    return 0.0;
  };

}
