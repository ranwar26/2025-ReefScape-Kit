// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.MotorConfigs.IntakeConfig;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

  private SparkMax m_intakeMotor;
  private RelativeEncoder m_encoder;

  public IntakeIOReal() {

    this.m_intakeMotor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
    this.m_encoder = m_intakeMotor.getEncoder();

    this.m_intakeMotor.configure(IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = this.m_encoder.getPosition() % (2.0 * Math.PI);
    inputs.velocity = this.m_encoder.getVelocity();
    inputs.appliedVolts =
        this.m_intakeMotor.getAppliedOutput() * this.m_intakeMotor.getBusVoltage();
    inputs.currentAmps = this.m_intakeMotor.getOutputCurrent();
  }

  @Override
  public void setIntakeVolts(double volts) {
    this.m_intakeMotor.setVoltage(volts);
  }

}
