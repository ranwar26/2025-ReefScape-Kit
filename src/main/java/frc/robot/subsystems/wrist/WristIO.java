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

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristVolts(double volts) {}

  public default double getCurrentPosition() {
    return 0.0;
  }
}
