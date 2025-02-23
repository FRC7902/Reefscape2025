// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralIndexerConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralIndexerSubsystem extends SubsystemBase {

    /** SparkMax motor controller object. */
    public SparkMax m_motor = new SparkMax(Constants.CoralIndexerConstants.kIndexMotorCAN,
            SparkMax.MotorType.kBrushless);

    /** Spark Max motor controller configuration object. */
    public SparkMaxConfig m_motorConfig = new SparkMaxConfig();

    /** Simple motor feedforward object. */
    public SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.CoralIndexerConstants.kS, Constants.CoralIndexerConstants.kV);

    /** Relative encoder object. */
    private RelativeEncoder m_encoder;

    /** Beam sensor to detect if the coral is present. */
    private DigitalInput m_beamSensor;

    /** Debouncer for the beam sensor input to prevent false triggering. */
    private Debouncer m_debouncedBeamBreak;

    /** Indexer motor speed. */
    public double m_indexSpeed = 0;

    /** Motor voltage for controlling the motor. */
    public double m_volts = 0;

    /** Simulated motor velocity for simulation. */
    private double m_simMotorVelocity = 0;

    /** Creates a new CoralIndexerSubsystem. */
    public CoralIndexerSubsystem() {

        // Configure the motor controller
        m_motorConfig.smartCurrentLimit(30).openLoopRampRate(CoralIndexerConstants.kRampRate)
                .idleMode(IdleMode.kBrake);

        // Apply the configuration to the motor controller
        m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        // Get the encoder object from the motor controller
        m_encoder = m_motor.getEncoder();

        // Get the beam sensor object
        m_beamSensor = new DigitalInput(Constants.CoralIndexerConstants.kBeamSensorPort);

        // Create a debouncer object for the beam sensor
        m_debouncedBeamBreak = new Debouncer(1);
    }

    /**
     * Sets the speed of the coral indexer.
     * 
     * @param speed The speed to set the coral indexer to
     */
    public void setSpeed(double speed) {
        m_indexSpeed = speed;
        m_motor.setVoltage(m_feedforward.calculate(m_indexSpeed));
    }

    /**
     * Sets the power of the coral indexer.
     * 
     * @param motorPower The power to set the coral indexer to
     */
    public void setPower(double motorPower) {
        m_motor.set(motorPower);
    }

    /**
     * Gets the speed of the coral indexer.
     * 
     * @return The speed of the coral indexer
     */
    public double getSpeed() {
        return m_indexSpeed;
    }

    /** Stops the motor. */
    public void stop() {
        setSpeed(0);
        m_motor.stopMotor();
    }

    /** Sets the idle mode of the motor controller to coast. */
    public void coast() {
        m_motorConfig.idleMode(IdleMode.kCoast);
    }

    /** Sets the idle mode of the motor controller to brake. */
    public void brake() {
        m_motorConfig.idleMode(IdleMode.kBrake);
    }

    /**
     * Indicates if the beam sensor is broken.
     * 
     * @return True if the beam sensor is broken
     */
    public boolean isBeamBroken() {
        return !m_beamSensor.get();
    }

    /**
     * Indicates if the subsystem has a coral.
     * 
     * @return True if the coral has broken the beam for a significant amount of time
     */
    public boolean hasCoral() {
        return m_debouncedBeamBreak.calculate(isBeamBroken());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update SmartDashboard
        SmartDashboard.putNumber("Index Speed", m_indexSpeed);
        SmartDashboard.putNumber("Motor Velocity (Encoder)", m_encoder.getVelocity());
        SmartDashboard.putBoolean("Beam Sensor Broken", isBeamBroken());

        // m_debouncedBeamBreak.calculate(m_beamSensor.get());
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("Encoder Reading", m_encoder.getPosition());

        SmartDashboard.putBoolean("Index Stopped", m_motor.getAppliedOutput() == 0);

        SmartDashboard.putNumber("Motor Voltage", m_motor.getBusVoltage());
        // SmartDashboard.putNumber("Motor Current", indexMotor.getMotorCurrent());

        SmartDashboard.putNumber("Index Setpoint", m_indexSpeed);

        SmartDashboard.putNumber("Applied Output", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("Index Velocity", m_encoder.getVelocity());

        double dt = 0.02;
        double tau = 0.5;
        double deltaV = (m_indexSpeed - m_simMotorVelocity) * dt / tau;
        m_simMotorVelocity += deltaV;
        SmartDashboard.putNumber("Simulated Motor Velocity", m_simMotorVelocity);
    }

}
