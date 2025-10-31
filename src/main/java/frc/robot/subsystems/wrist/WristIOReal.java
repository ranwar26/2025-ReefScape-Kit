// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.WristConstants;
import frc.robot.MotorConfigs.WristConfig;

/** Add your docs here. */
public class WristIOReal implements WristIO {

  private SparkMax m_wristMotor;
  private RelativeEncoder m_encoder;

  private PIDController m_wristPIDController;

  private double targetAngle = 0.0;

  public WristIOReal() {

    this.m_wristMotor = new SparkMax(WristConstants.kMotorID, MotorType.kBrushless);
    this.m_encoder = m_wristMotor.getEncoder();

    this.m_wristMotor.configure(WristConfig.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.m_wristPIDController = new PIDController(
      WristConstants.kRealP,
      WristConstants.kRealI,
      WristConstants.kRealD
    );
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.position = this.m_encoder.getPosition();
    inputs.targetPosition = this.targetAngle;
    inputs.errorPosition = Math.abs(this.targetAngle - this.m_encoder.getPosition());
    inputs.velocity = this.m_encoder.getVelocity();
    inputs.appliedVolts = this.m_wristMotor.getAppliedOutput() * this.m_wristMotor.getBusVoltage();
    inputs.currentAmps = this.m_wristMotor.getOutputCurrent();
  }

  @Override
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
    double speed = this.m_wristPIDController.calculate(this.m_encoder.getPosition(), angle);
    double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);
    volts += WristConstants.kRealG;

    this.m_wristMotor.setVoltage(volts);
  }

  @Override
  public void resetPID() {
    this.m_wristPIDController.reset();
  }

}
