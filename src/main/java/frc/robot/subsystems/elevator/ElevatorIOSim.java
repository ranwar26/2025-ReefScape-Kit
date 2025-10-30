// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim elevatorSim;

  private PIDController m_elevatorPIDController;

  private double appliedVolts;
  private double targetLength;

  public ElevatorIOSim() {

    this.elevatorSim = new ElevatorSim(
      DCMotor.getNEO(2),
      ElevatorConstants.motorToDrumRatio,
      13.0,
      0.06,
      0.659,
      2.0,
      true,
      0.659
      );

    this.m_elevatorPIDController =new PIDController(
        ElevatorConstants.kSimP,
        ElevatorConstants.kSimI,
        ElevatorConstants.kSimD
    );
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    this.elevatorSim.setInputVoltage(this.appliedVolts);
    this.elevatorSim.update(0.02);

    inputs.position = this.elevatorSim.getPositionMeters();
    inputs.targetPosition = this.targetLength;
    inputs.errorPosition = Math.abs(this.targetLength - this.elevatorSim.getPositionMeters());
    inputs.velocity = this.elevatorSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = this.appliedVolts;
    inputs.currentAmps = this.elevatorSim.getCurrentDrawAmps();

  }

  @Override
  public void setTargetLength(double length) {
    this.targetLength = length;
    double speed = this.m_elevatorPIDController.calculate(getCurrentLength() - length);
    this.appliedVolts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);
    this.appliedVolts += ElevatorConstants.kSimG;

  }

  @Override
    public double getTargetLength() {
        return this.targetLength;
    }

  @Override
  public double getCurrentLength() {
    return this.elevatorSim.getPositionMeters();
  }

  @Override
  public void resetPID() {
      this.m_elevatorPIDController.reset();
  }

}
