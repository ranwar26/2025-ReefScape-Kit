// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {

        public double position = 0.0;
        public double targetPosition = 0.0;
        public double errorPosition = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

    }

    /**
     * Updates the inputs of the elevator subsystem
     * 
     * @param inputs - the class to be updated
     */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /**
     * Sets the target value for the elevator to move to
     * 
     * @param length - the target length
     */
    public default void setTargetLength(double length) {
    }

    /**
     * gets the current target of the elevator
     * 
     * @return - the target length
     */
    public default double getTargetLength() {
        return 0.0;
    }

    /**
     * gets the current length of the elevator
     * 
     * @return - the current length of the side picked
     */
    public default double getCurrentLength() {
        return 0.0;
    };

    /**
     * resets the PID controller
     */
    public default void resetPID() {
    }

}
