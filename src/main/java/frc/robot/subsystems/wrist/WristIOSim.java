// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.WristConsants;

/** Add your docs here. */
public class WristIOSim implements WristIO {

  private DCMotorSim m_wristMotor;

  private LoggedMechanism2d mechanism = new LoggedMechanism2d(6, 3);
  private LoggedMechanismLigament2d gripper = new LoggedMechanismLigament2d("gripper", 0.2, 0.0);

  private double appliedVolts;

  public WristIOSim() {

    this.m_wristMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WristConsants.motorGearbox, 0.205, WristConsants.motorToWheelRatio),
            WristConsants.motorGearbox);

    this.mechanism.getRoot("root", 3.0, 1.5).append(gripper);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    this.m_wristMotor.setInputVoltage(appliedVolts);
    this.m_wristMotor.update(0.02);

    inputs.position = this.m_wristMotor.getAngularPositionRad();
    inputs.velocity = this.m_wristMotor.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = this.m_wristMotor.getCurrentDrawAmps();

    gripper.setAngle(Math.toDegrees(inputs.position));
    Logger.recordOutput("MyMechanism", mechanism);
  }

  @Override
  public void setWristVolts(double volts) {
    appliedVolts = volts;
  }

}
