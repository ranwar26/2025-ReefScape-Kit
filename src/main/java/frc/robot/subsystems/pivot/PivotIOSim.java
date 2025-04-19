// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.PivotConstants;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {

    private DCMotorSim m_leftMotor;
    private DCMotorSim m_rightMotor;

    private double appliedVoltsLeft;
    private double appliedVoltsRight;

    public PivotIOSim() {

        this.m_leftMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        PivotConstants.motorGearbox, 5.0, PivotConstants.motorToWheelRatio),
                PivotConstants.motorGearbox);
        this.m_rightMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        PivotConstants.motorGearbox, 5.0, PivotConstants.motorToWheelRatio),
                PivotConstants.motorGearbox);

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        this.m_leftMotor.setInputVoltage(appliedVoltsLeft);
        this.m_leftMotor.update(0.02);

        inputs.leftPosition = this.m_leftMotor.getAngularPositionRad();
        inputs.leftVelocity = this.m_leftMotor.getAngularVelocityRadPerSec();
        inputs.leftAppliedVolts = appliedVoltsLeft;
        inputs.leftCurrentAmps = this.m_leftMotor.getCurrentDrawAmps();

        this.m_rightMotor.setInputVoltage(appliedVoltsRight);
        this.m_rightMotor.update(0.02);

        inputs.rightPosition = this.m_rightMotor.getAngularPositionRad();
        inputs.rightVelocity = this.m_rightMotor.getAngularVelocityRadPerSec();
        inputs.rightAppliedVolts = appliedVoltsRight;
        inputs.rightCurrentAmps = this.m_rightMotor.getCurrentDrawAmps();
    }

    @Override
    public void setPivotVolts(double volts) {
        appliedVoltsLeft = volts;
        appliedVoltsRight = -volts;
    }

    @Override
    public double getCurrentAngle(String side) {
        switch (side) {
            case "Right":
                return this.m_rightMotor.getAngularPositionRad();
        
            default:
                return this.m_leftMotor.getAngularPositionRad();
        }
    }

}
