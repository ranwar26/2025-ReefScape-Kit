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

/** Add your docs here. */
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
        Math.toRadians(30.0)
        );

        this.m_pivotPIDController = new PIDController(
            PivotConstants.kSimP,
            PivotConstants.kSimI,
            PivotConstants.kSimD
        );

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {

        this.pivotArmSim.setInputVoltage(appliedVolts);
        this.pivotArmSim.update(0.02);

        inputs.position = pivotArmSim.getAngleRads();
        inputs.targetPosition = this.targetAngle;
        inputs.errorPosition = Math.abs(inputs.targetPosition - inputs.position);
        inputs.velocity = pivotArmSim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = this.pivotArmSim.getCurrentDrawAmps();
    }

    @Override
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
        double speed = this.m_pivotPIDController.calculate(getCurrentAngle(), angle);
        double volts = 12.0 * MathUtil.clamp(speed, -1.0, 1.0);

        //If the pivot is under 0.1 degrees of error AND trying to apply more than 6 volt, then don't apply those 6 volts
        if(MathUtil.isNear(this.pivotArmSim.getAngleRads(), angle, Math.toRadians(0.1), 0.0, 2.0 * Math.PI) && volts > 6.0)
            volts = 0.0;

        this.appliedVolts = volts;
    }

    @Override
    public double getTargetAngle() {
        return this.targetAngle;
    }

    @Override
    public double getCurrentAngle() {
        return this.pivotArmSim.getAngleRads();
    }

}
