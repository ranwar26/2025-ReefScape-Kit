// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.WristConstants;

/** Add your docs here. */
public class WristIOSim implements WristIO {

  private DCMotorSim m_wristMotor;

  private double appliedVolts;

  public WristIOSim() {

    this.m_wristMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WristConstants.motorGearbox, 0.205, WristConstants.motorToWheelRatio),
            WristConstants.motorGearbox);

  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    this.m_wristMotor.setInputVoltage(appliedVolts);
    this.m_wristMotor.update(0.02);

    inputs.position = this.m_wristMotor.getAngularPositionRad();
    inputs.velocity = this.m_wristMotor.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = this.m_wristMotor.getCurrentDrawAmps();

  }

  @Override
  public void setWristVolts(double volts) {
    appliedVolts = volts;
  }

  @Override
  public double getCurrentAngle() {
    return this.m_wristMotor.getAngularPositionRad();
  }

}
