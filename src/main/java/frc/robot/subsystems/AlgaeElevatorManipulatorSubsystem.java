// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlgaeElevatorManipulatorConstants;

public class AlgaeElevatorManipulatorSubsystem extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(AlgaeElevatorManipulatorConstants.kMotorCANId, MotorType.kBrushless);
  private SparkMaxConfig m_motorConfig = new SparkMaxConfig();

  private DigitalInput m_beamBreak = new DigitalInput(AlgaeElevatorManipulatorConstants.kbeamBreakPortId);

  /** Creates a new AlgaeElevatorManipulatorSubsystem. */
  public AlgaeElevatorManipulatorSubsystem() {
    configure();
  }

  private void configure() {
    m_motorConfig.smartCurrentLimit(AlgaeElevatorManipulatorConstants.kMotorCurrentLimit);
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  public void stopIntake() {
    m_motor.stopMotor();
  }

  public boolean hasAlgae() {
    return m_beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hasAlgae", hasAlgae());
  }
}
