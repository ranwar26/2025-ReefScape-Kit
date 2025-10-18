// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public double position = 0.0;
        public double targetPosition = 0.0;
        public double errorPosition = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

    }

    /**
   * Tell the pivot to move to the target length
   * 
   * @param angle - the target angle
   */
    public default void updateInputs(PivotIOInputs inputs) {
    }

    /**
     * Sets the motors at the given voltage
     * 
     * @param volts - the number of volt
     */
    public default void setTargetAngle(double angle) {
    }

    
    public default double getTargetAngle() {
        return 0.0;
    }

    /**
     * get the current angle of the pivot
     * 
     * @return - the current angle of the pivot
     */
    public default double getCurrentAngle() {
        return 0.0;
    };

    /**
     * resets the PID controller
     */
    public default void resetPID() {
    }
}
