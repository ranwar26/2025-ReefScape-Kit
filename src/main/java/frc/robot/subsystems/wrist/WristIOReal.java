// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class WristIOReal implements WristIO {

  private SparkMax m_wristMotor;
  private RelativeEncoder m_encoder;

  public WristIOReal() {

    this.m_wristMotor = new SparkMax(0, MotorType.kBrushless); // TODO: set motor ID
    this.m_encoder = m_wristMotor.getEncoder();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.position = this.m_encoder.getPosition();
    inputs.velocity = this.m_encoder.getVelocity();
    inputs.appliedVolts = this.m_wristMotor.getAppliedOutput() * this.m_wristMotor.getBusVoltage();
    inputs.currentAmps = this.m_wristMotor.getOutputCurrent();
  }

  @Override
  public void setWristVolts(double volts) {
    this.m_wristMotor.setVoltage(volts);
  }

  @Override
  public double getCurrentAngle() {
    return this.m_encoder.getPosition();
  }

}
