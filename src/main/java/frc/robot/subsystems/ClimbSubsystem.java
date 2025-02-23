// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {

    /** Spark Max leader motor controller object. */
    private final SparkMax m_leaderMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    
    /** Spark Max follower motor controller object. */
    private final SparkMax m_followerMotor = new SparkMax(ClimbConstants.kClimbFollowerMotorCANID, MotorType.kBrushless);

    /** Spark Max leader motor controller simulation object. */
    private final SparkMaxSim s_simLeaderMotor = new SparkMaxSim(m_leaderMotor, DCMotor.getNeo550(2));

    /** Spark Max leader motor controller configuration object. */
    private final SparkMaxConfig m_leaderMotorConfig = new SparkMaxConfig();

    /** Spark Max follower motor controller configuration object. */
    private final SparkMaxConfig m_followerMotorConfig = new SparkMaxConfig();

    /** Absolute encoder object. */
    private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(ClimbConstants.kRevThroughBoreIO, 0, 0);

    /** Servo object for the left funnel servo. */        
    private final Servo m_leftServo = new Servo(ClimbConstants.kLeftServoID);

    /** Servo object for the right funnel servo. */
    private final Servo m_rightServo = new Servo(ClimbConstants.kRightServoID);

    /** Indicates whether the funnel is unlocked. */
    private boolean isFunnelUnlocked;

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {

        // Clear any faults that may have been stored in the motor controllers
        m_leaderMotor.clearFaults();
        m_followerMotor.clearFaults();

        // Sets the follower motor to follow the leader motor
        m_followerMotorConfig.follow(m_leaderMotor);

        // Invert the motors
        m_leaderMotorConfig.inverted(false);
        m_followerMotorConfig.inverted(false);

        // Sets the idle mode of the motors to brake when no power is being applied 
        m_leaderMotorConfig.idleMode(IdleMode.kBrake);
        m_followerMotorConfig.idleMode(IdleMode.kBrake);

        // Set current limits for the motors
        m_leaderMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent,
                ClimbConstants.kMotorFreeSpeedCurrent);
        m_followerMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent,
                ClimbConstants.kMotorFreeSpeedCurrent);

        // Invert the encoder
        m_absoluteEncoder.setInverted(false);

        // Apply the configurations to the motors
        m_leaderMotor.configure(m_leaderMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_followerMotor.configure(m_followerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        lockFunnel();
        isFunnelUnlocked = false;
    }

    /** Sets the voltage of the motor.
     * 
     * @param voltage The voltage to set the motor to.
     */
    public void setVoltage(double voltage) {
        m_leaderMotor.setVoltage(voltage);
    }

    /**
     * Returns the encoder value.
     * 
     * @return The encoder value in rotations.
     */
    public double getEncoderDistance() {
        return m_absoluteEncoder.get();
    }

    /** Stops the motor. */
    public void stopMotors() {
        m_leaderMotor.stopMotor();
    }

    /** Locks the funnel. */
    public void lockFunnel() {
        m_leftServo.setAngle(80);
        m_rightServo.setAngle(95);
    }

    /** Unlocks the funnel. */
    public void unlockFunnel() {
        m_leftServo.setAngle(180);
        m_rightServo.setAngle(0);
        isFunnelUnlocked = true;
    }

    /** Stops the funnel servos. */
    public void stopFunnelServos() {
        m_leftServo.setAngle(90);
        m_rightServo.setAngle(90);
    }

    /**
     * Checks whether the funnel mechanism is unlocked.
     * 
     * @return True if the funnel is unlocked, false otherwise.
    */
    public boolean isFunnelUnlocked() {
        return isFunnelUnlocked;
    }

    /**
     * Checks the motor for faults and returns the type of error encountered.
     * <p>
     * The method checks for common motor faults such as communication issues, firmware errors, 
     * or temperature problems. If no fault is detected, it returns {@code null}.
     * 
     * @param motor The motor to check for errors.
     * @return A string representing the type of fault, or {@code null} if no fault is present.
     */
    public String reportMotorError(SparkMax motor) {
        Faults motorFault = motor.getFaults();
        if (motorFault.can) {
            return "CAN";
        } else if (motorFault.escEeprom) {
            return "ESC_EEPROM";
        } else if (motorFault.firmware) {
            return "FIRMWARE";
        } else if (motorFault.gateDriver) {
            return "GATE_DRIVER";
        } else if (motorFault.motorType) {
            return "MOTOR_TYPE";
        } else if (motorFault.sensor) {
            return "SENSOR";
        } else if (motorFault.temperature) {
            return "TEMPERATURE";
        }
        return null;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Updates the SmartDashboard
        if ((!Robot.isSimulation())) {
            SmartDashboard.putNumber("Encoder reading", m_absoluteEncoder.get());
            SmartDashboard.putBoolean("Climb stopped", m_leaderMotor.get() == 0);

            SmartDashboard.putNumber("Leader Motor Speed", m_leaderMotor.get());
            SmartDashboard.putNumber("Leader Motor Voltage", m_leaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Leader Motor Current", m_leaderMotor.getOutputCurrent());
            SmartDashboard.putNumber("Leader Motor Temperature",
                    m_leaderMotor.getMotorTemperature());

            SmartDashboard.putNumber("Follower Motor Speed", m_followerMotor.get());
            SmartDashboard.putNumber("Follower Motor Voltage", m_followerMotor.getBusVoltage());
            SmartDashboard.putNumber("Follower Motor Current", m_followerMotor.getOutputCurrent());
            SmartDashboard.putNumber("Follower Motor Temperature",
                    m_followerMotor.getMotorTemperature());

            if (m_leaderMotor.hasActiveFault()) {
                DriverStation.reportWarning(
                        "MOTOR WARNING: SparkMax ID " + ClimbConstants.kClimbLeaderMotorCANID
                                + " is currently reporting an error with: \""
                                + reportMotorError(m_leaderMotor) + "\"",
                        true);
            }
            if (m_followerMotor.hasActiveFault()) {
                DriverStation.reportWarning(
                        "MOTOR WARNING: SparkMax ID " + ClimbConstants.kClimbFollowerMotorCANID
                                + " is currently reporting an error with: \""
                                + reportMotorError(m_followerMotor) + "\"",
                        true);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // Displays live motor and limit switch metrics on SmartDashboard
        if (Robot.isSimulation()) {
            // SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
            SmartDashboard.putBoolean("Climb stopped", s_simLeaderMotor.getAppliedOutput() == 0);
            SmartDashboard.putNumber("Motor Bus Voltage", s_simLeaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Motor Current", s_simLeaderMotor.getMotorCurrent());
            SmartDashboard.putNumber("Climb Setpoint", s_simLeaderMotor.getSetpoint());
            SmartDashboard.putNumber("Applied Output", s_simLeaderMotor.getAppliedOutput());
            SmartDashboard.putNumber("Climb Velocity", s_simLeaderMotor.getVelocity());
            SmartDashboard.putNumber("Climb Position", s_simLeaderMotor.getPosition());

            // Checks to see if motors are going upwards and if the encoder has reached the
            // set limit

            s_simLeaderMotor.iterate(m_leaderMotor.getAppliedOutput(), 12, 0.02);
            // m_climbSim.setInput(s_climbLeaderMotor.getVelocity() *
            // RobotController.getBatteryVoltage());
            // m_climbSim.update(0.020);
            // s_absoluteEncoder.set(m_climbSim.getPositionMeters());
            // RoboRioSim.setVInVoltage(
            // BatterySim.calculateDefaultBatteryLoadedVoltage(m_climbSim.getCurrentDrawAmps()));
        }
    }
}
