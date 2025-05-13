// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim elevatorSim;

  private double appliedVolts;

  public ElevatorIOSim() {

    this.elevatorSim = new ElevatorSim(
      DCMotor.getNEO(2),
      ElevatorConstants.motorToWheelRatio,
      8.02,
      0.06,
      0.0,
      0.5,
      true,
      0.0
      );
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    this.elevatorSim.setInputVoltage(appliedVolts);
    this.elevatorSim.update(0.02);

    inputs.position = this.elevatorSim.getPositionMeters();
    inputs.velocity = this.elevatorSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = this.elevatorSim.getCurrentDrawAmps();

  }

  @Override
  public void setElevatorVolts(double volts) {
    this.appliedVolts = volts;
  }

  @Override
  public double getCurrentLength() {
    return this.elevatorSim.getPositionMeters();
  }

}
