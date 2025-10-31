// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

/**
 * The sim implementation of the pivot
 */
public class PivotIOSim implements PivotIO {

	private SingleJointedArmSim pivotArmSim;

	private PIDController m_pivotPIDController;

	private double targetAngle;
	private double appliedVolts;

	public PivotIOSim() {

		this.pivotArmSim = new SingleJointedArmSim(
				DCMotor.getNeo550(2),
				PivotConstants.motorToPivotAngleRatio,
				0.970,
				ElevatorConstants.kHomeLength,
				Math.toRadians(30.0),
				Math.toRadians(225.0),
				true,
				Math.toRadians(30.0));

		this.m_pivotPIDController = new PIDController(
				PivotConstants.kSimP,
				PivotConstants.kSimI,
				PivotConstants.kSimD);

	}

	@Override
	public void updateInputs(PivotIOInputs inputs) {

		this.pivotArmSim.setInputVoltage(appliedVolts);
		this.pivotArmSim.update(0.02);

		inputs.currentPosition = pivotArmSim.getAngleRads();
		inputs.targetPosition = this.targetAngle;
		inputs.errorPosition = Math.abs(inputs.targetPosition - inputs.currentPosition);
		inputs.velocity = pivotArmSim.getVelocityRadPerSec();
		inputs.appliedVolts = appliedVolts;
		inputs.currentAmps = this.pivotArmSim.getCurrentDrawAmps();
	}

	@Override
	public void setTargetAngle(double angle) {
		this.targetAngle = angle;
		double speed = this.m_pivotPIDController.calculate(this.pivotArmSim.getAngleRads(), angle);
		double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);

		this.appliedVolts = volts;
	}

	@Override
	public void resetPID() {
		this.m_pivotPIDController.reset();
	}
}
