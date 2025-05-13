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
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public double leftPosition = 0.0;
        public double leftVelocity = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public double rightPosition = 0.0;
        public double rightVelocity = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    /**
     * Updates the inputs of the elevator subsystem
     * 
     * @param inputs - the class to be updated
     */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /**
     * Sets the motors at the given voltage
     * 
     * @param volts - the number of volt
     */
    public default void setElevatorVolts(double volts) {
    }

    /**
     * get the current length of the elevator
     * 
     * @param side - the side to grab data from
     * @return - the current length of the side picked
     */
    public default double getCurrentLength() {
        return 0.0;
    };

}
