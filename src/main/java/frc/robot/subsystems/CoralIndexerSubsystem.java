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
    public SparkMax m_indexMotor = new SparkMax(Constants.CoralIndexerConstants.kIndexMotorCAN,
            SparkMax.MotorType.kBrushless);
    public SparkMaxConfig m_indexMotorConfig = new SparkMaxConfig();
    public SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
            Constants.CoralIndexerConstants.kS, Constants.CoralIndexerConstants.kV);

    private RelativeEncoder m_encoder;
    private DigitalInput m_beamSensor;
    private Debouncer m_debouncedBeamBreak;

    public double m_indexSpeed = 0;
    public double m_volts = 0;

    // Simulation variable: a simple model of motor velocity (units matching your
    // kVelocity)
    private double m_simMotorVelocity = 0;

    public CoralIndexerSubsystem() {
        m_indexMotorConfig.smartCurrentLimit(30).openLoopRampRate(CoralIndexerConstants.kRampRate)
                .idleMode(IdleMode.kBrake);

        m_indexMotor.configure(m_indexMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        m_encoder = m_indexMotor.getEncoder();
        m_beamSensor = new DigitalInput(Constants.CoralIndexerConstants.kBeamSensorPort);
        m_debouncedBeamBreak = new Debouncer(1);
    }

    public void setSpeed(double speed) {
        m_indexSpeed = speed;
        m_indexMotor.setVoltage(m_feedforward.calculate(m_indexSpeed));
    }

    public void setPower(double motorPower) {
        m_indexMotor.set(motorPower);
    }

    public double getSpeed() {
        return m_indexSpeed;
    }

    public void stop() {
        setSpeed(0);
        m_indexMotor.stopMotor();
    }

    public void coast() {
        m_indexMotorConfig.idleMode(IdleMode.kCoast);
    }

    public void brake() {
        m_indexMotorConfig.idleMode(IdleMode.kBrake);
    }

    public boolean isBeamBroken() {
        return !m_beamSensor.get();
    }

    /**
     * Indicates if the subsystem has a coral
     * 
     * @return true if the coral has broken the beam for a significant amount of time
     */
    public boolean hasCoral() {
        return m_debouncedBeamBreak.calculate(isBeamBroken());
    }

    @Override
    public void periodic() {
        // This code runs in both real and simulation modes.
        // SmartDashboard.putNumber("Index Speed", m_indexSpeed);
        // SmartDashboard.putNumber("Motor Velocity (Encoder)", m_encoder.getVelocity());
        //SmartDashboard.putBoolean("Beam Sensor Broken", isBeamBroken()); commented out for testing

        // m_debouncedBeamBreak.calculate(m_beamSensor.get());



    }

    @Override
    public void simulationPeriodic() {
        //SmartDashboard.putNumber("Encoder Reading", m_encoder.getPosition()); commented out for testing

        // SmartDashboard.putBoolean("Index Stopped", m_indexMotor.getAppliedOutput() == 0);

        // SmartDashboard.putNumber("Motor Voltage", m_indexMotor.getBusVoltage());
        // // SmartDashboard.putNumber("Motor Current", indexMotor.getMotorCurrent());

        // SmartDashboard.putNumber("Index Setpoint", m_indexSpeed);

        // SmartDashboard.putNumber("Applied Output", m_indexMotor.getAppliedOutput());
        // SmartDashboard.putNumber("Index Velocity", m_encoder.getVelocity());

        double dt = 0.02;
        double tau = 0.5;
        double deltaV = (m_indexSpeed - m_simMotorVelocity) * dt / tau;
        m_simMotorVelocity += deltaV;
        // SmartDashboard.putNumber("Simulated Motor Velocity", m_simMotorVelocity);
    }

}
