// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.PivotConstants;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {

    private DCMotorSim m_leftMotor;
    private DCMotorSim m_rightMotor;

    private double appliedVoltsLeft;
    private double appliedVoltsRight;

    private LoggedMechanism2d mechanism = new LoggedMechanism2d(6, 3);
    private LoggedMechanismLigament2d elevator = new LoggedMechanismLigament2d("elevator", 1.0, 0.0);

    public PivotIOSim() {

        this.m_leftMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        PivotConstants.motorGearbox, 0.8, PivotConstants.motorToWheelRatio),
                PivotConstants.motorGearbox);
        this.m_rightMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        PivotConstants.motorGearbox, 0.8, PivotConstants.motorToWheelRatio),
                PivotConstants.motorGearbox);

        this.mechanism.getRoot("root", 3.0, 1.5).append(elevator);
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

        elevator.setAngle(Math.toDegrees(inputs.leftPosition));
        Logger.recordOutput("pivot", mechanism);
    }

    @Override
    public void setPivotVolts(double volts) {
        appliedVoltsLeft = volts;
        appliedVoltsRight = -volts;
    }

}
