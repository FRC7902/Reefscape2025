// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManipulatorConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

    private TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;


    private DigitalInput m_beamBreak;

    /** Creates a new AlgaeElevatorManipulatorSubsystem. */
    public AlgaeManipulatorSubsystem() {
        m_motorConfig = new TalonFXConfiguration();
        m_motor = new TalonFX(AlgaeManipulatorConstants.kMotorCANId);
        m_beamBreak = new DigitalInput(AlgaeManipulatorConstants.kbeamBreakPortId);
        configure();
    }

    private void configure() {
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit =
                AlgaeManipulatorConstants.kMotorStatorCurrentLimit;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit =
                AlgaeManipulatorConstants.kMotorSupplyCurrentLimit;

        m_motor.getConfigurator().apply(m_motorConfig);
    }

    public void setIntakeVoltage(double voltage) {

        // TODO: For safety, if moving, check current for some duration (e.g., 0.5 sec),
        // if current is above threshold, stop motor. Due to current limit of 60A, we do
        // not want the motor to be running for more than 75s.

        m_motor.setVoltage(voltage);
    }

    public void stopIntake() {
        setIntakeVoltage(0);
        m_motor.stopMotor();
    }

    public boolean hasAlgae() {
        return m_beamBreak.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putBoolean("hasAlgae", hasAlgae()); commented out for testing
    }
}
