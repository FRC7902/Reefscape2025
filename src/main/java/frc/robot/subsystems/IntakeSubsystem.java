// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.intakeCANid);
  /** Creates a new IntakeSubsystem. 
   * @param m_operatorStick */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
