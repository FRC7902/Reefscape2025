// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManipulatorConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

    private SparkMax m_motor =
            new SparkMax(AlgaeManipulatorConstants.kMotorCANId, MotorType.kBrushless);
    private SparkMaxConfig m_motorConfig = new SparkMaxConfig();

    private DigitalInput m_beamBreak = new DigitalInput(AlgaeManipulatorConstants.kbeamBreakPortId);

    /** Creates a new AlgaeElevatorManipulatorSubsystem. */
    public AlgaeManipulatorSubsystem() {
        configure();
    }

    private void configure() {
        m_motorConfig.smartCurrentLimit(AlgaeManipulatorConstants.kMotorCurrentLimit);
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setIntakeVoltage(double voltage) {

        // TODO: For safety, if moving, check current for some duration (e.g., 0.5 sec),
        // if current is above threshold, stop motor. Due to current limit of 60A, we do
        // not want the motor to be running for more than 75s.

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
