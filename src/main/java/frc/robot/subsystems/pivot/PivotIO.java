// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public double leftPosition = 0.0;
        public double leftVelocity = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public double rightPosition = 0.0;
        public double rightVelocity = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setPivotVolts(double volts) {}

    public default double getCurrentAngle(String side) {return 0.0;};
}
