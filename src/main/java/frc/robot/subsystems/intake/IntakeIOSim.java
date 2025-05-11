// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private DCMotorSim m_intakeMotor;

  private double appliedVolts;

  public IntakeIOSim() {

    this.m_intakeMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.motorGearbox, 0.025, IntakeConstants.motorToWheelRatio),
            IntakeConstants.motorGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.m_intakeMotor.setInputVoltage(appliedVolts);
    this.m_intakeMotor.update(0.02);

    inputs.position = this.m_intakeMotor.getAngularPositionRad();
    inputs.velocity = this.m_intakeMotor.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = this.m_intakeMotor.getCurrentDrawAmps();
  }

  @Override
  public void setIntakeVolts(double volts) {
    appliedVolts = volts;
  }

  @Override
  public double getCurrentVolts() {
    return appliedVolts;
  }

  @Override
  public double getCurrentAngle() {
    return this.m_intakeMotor.getAngularPositionRad();
  }
}
