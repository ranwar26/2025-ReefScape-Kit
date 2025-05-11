// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {

    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Set the intake target speed
   * 
   * @param percentage - the percentage of intake's speed
   */
  public void setIntakePercentage(double percentage) {
    this.io.setIntakeVolts(MathUtil.clamp(percentage, -1.0, 1.0) * 12.0);
  }

  /**
   * Returns the current Volts
   * 
   * @return double - the volts
   */
  public double getCurrentVolts() {
    return this.io.getCurrentVolts();
  }

  /**
   * Gets the current angle of the intake
   * (Not very useful, but useful for see if the intake is moving)
   * 
   * @return - the current angle of the intake
   */
  public double getCurrentAngle() {
    return this.io.getCurrentAngle();
  }
}
