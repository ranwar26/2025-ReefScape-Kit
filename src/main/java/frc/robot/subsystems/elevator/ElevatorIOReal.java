// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.MotorConfigs.ElevatorConfig;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO {

    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;

    private PIDController m_elevatorPIDController;

    private RelativeEncoder m_rightEncoder;

    private double targetLength;

    public ElevatorIOReal() {

        this.m_leftMotor = new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
        this.m_rightMotor = new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

        tryUntilOk(
            m_leftMotor,
            5,
            () -> this.m_leftMotor.configure(ElevatorConfig.leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
        tryUntilOk(
            m_rightMotor,
            5,
            () -> this.m_rightMotor.configure(ElevatorConfig.rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );

        this.m_rightEncoder = this.m_rightMotor.getEncoder();
        tryUntilOk(
            m_rightMotor,
            5,
            () -> this.m_rightEncoder.setPosition(ElevatorConstants.kHomeLength)
        );

        this.m_elevatorPIDController = new PIDController(
            ElevatorConstants.kRealP,
            ElevatorConstants.kRealI,
            ElevatorConstants.kRealD
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = this.m_rightEncoder.getPosition();
        inputs.targetPosition = this.targetLength;
        inputs.errorPosition = Math.abs(this.targetLength - this.m_rightEncoder.getPosition());
        inputs.velocity = this.m_rightEncoder.getVelocity();
        inputs.appliedVolts = this.m_rightMotor.getAppliedOutput() * this.m_rightMotor.getBusVoltage();
        inputs.currentAmps = this.m_rightMotor.getOutputCurrent();
    }

    @Override
    public void setTargetLength(double length) {
        this.targetLength = length;
        double speed = this.m_elevatorPIDController.calculate(this.m_rightEncoder.getPosition(), length);
        double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);

        volts += ElevatorConstants.kRealG;

        this.m_leftMotor.setVoltage(volts);
        this.m_rightMotor.setVoltage(volts);
    }

    @Override
    public void resetPID() {
        this.m_elevatorPIDController.reset();
    }

}
