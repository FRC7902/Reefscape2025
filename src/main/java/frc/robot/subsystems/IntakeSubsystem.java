// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.intakeCANid, MotorType.kBrushless);
  private final DigitalInput intakeSensor = new DigitalInput(IntakeConstants.beamBrakePort);

  private final XboxController m_operatorStick;
  public boolean beamBrakeOverride = false;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(XboxController operatorStick) {
    m_operatorStick = operatorStick;
    stopMotor();
    m_intakeMotor.setInverted(true);
  }

  public void stopMotor() {
    m_intakeMotor.stopMotor();
  }

  public void setPower(double power) {
    m_intakeMotor.set(power);
  }

  public boolean getBeamBrake() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
