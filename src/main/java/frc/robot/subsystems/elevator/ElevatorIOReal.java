// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO {

    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    public ElevatorIOReal() {

        this.m_leftMotor = new SparkMax(0, MotorType.kBrushless); // TODO: set motor ID
        this.m_rightMotor = new SparkMax(0, MotorType.kBrushless);

        this.m_leftEncoder = this.m_leftMotor.getEncoder();
        this.m_rightEncoder = this.m_rightMotor.getEncoder();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftPosition = this.m_leftEncoder.getPosition();
        inputs.leftVelocity = this.m_leftEncoder.getVelocity();
        inputs.leftAppliedVolts = this.m_leftMotor.getAppliedOutput() * this.m_leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = this.m_leftMotor.getOutputCurrent();

        inputs.rightPosition = this.m_rightEncoder.getPosition();
        inputs.rightVelocity = this.m_rightEncoder.getVelocity();
        inputs.rightAppliedVolts = this.m_rightMotor.getAppliedOutput() * this.m_rightMotor.getBusVoltage();
        inputs.rightCurrentAmps = this.m_rightMotor.getOutputCurrent();
    }

    @Override
    public void setElevatorVolts(double volts) {
        this.m_leftMotor.setVoltage(volts);
        this.m_rightMotor.setVoltage(-volts);
    }

    @Override
    public double getCurrentLength() {
        return this.m_rightEncoder.getPosition();
    }

}
