// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class PivotIOReal implements PivotIO {

    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;

    private RelativeEncoder m_rightEncoder;

    private PIDController m_pivotPIDController;

    private double m_targetAngle;

    public PivotIOReal() {

        this.m_leftMotor = new SparkMax(PivotConstants.kLeftMotorID, MotorType.kBrushless);
        this.m_rightMotor = new SparkMax(PivotConstants.kRightMotorID, MotorType.kBrushless);

        this.m_rightEncoder = this.m_rightMotor.getEncoder();
        this.m_rightEncoder.setPosition(this.m_rightMotor.getAbsoluteEncoder().getPosition() * (Math.PI / 2.0));


        this.m_pivotPIDController = new PIDController(
        PivotConstants.kRealP,
        PivotConstants.kRealI,
        PivotConstants.kRealD
        );
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.position = this.m_rightEncoder.getPosition();
        inputs.targetPosition = this.m_targetAngle;
        inputs.errorPosition = Math.abs(this.m_targetAngle - this.m_rightEncoder.getPosition());
        inputs.velocity = this.m_rightEncoder.getVelocity();
        inputs.appliedVolts = this.m_rightMotor.getAppliedOutput() * this.m_rightMotor.getBusVoltage();
        inputs.currentAmps = this.m_rightMotor.getOutputCurrent();
    }

    @Override
    public void setTargetAngle(double angle) {
        this.m_targetAngle = angle;
        double speed = this.m_pivotPIDController.calculate(getCurrentAngle(), angle);
        double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);

        //If the pivot is under 0.1 degrees of error AND trying to apply more than 6 volt, then don't apply those 6 volts
        if(MathUtil.isNear(this.getCurrentAngle(), angle, Math.toRadians(0.1), 0.0, 2.0 * Math.PI) && volts > 6.0)
            volts = 0.0;

        this.m_leftMotor.setVoltage(-volts);
        this.m_rightMotor.setVoltage(volts);
    }

    @Override
    public double getTargetAngle() {
        return this.m_targetAngle;
    }

    @Override
    public double getCurrentAngle() {
        return this.m_rightEncoder.getPosition();
    }
}
