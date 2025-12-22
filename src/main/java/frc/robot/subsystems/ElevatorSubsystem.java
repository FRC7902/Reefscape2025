// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    /** Enum representing elevator positions */
    public enum ElevatorPosition {
        CORAL_L1, CORAL_L2, CORAL_L3, CORAL_STATION_AND_PROCESSOR, ALGAE_HIGH, ALGAE_LOW, UNKNOWN
    }

    /** TalonFX leader motor controller object */
    private final TalonFX m_leaderMotor;

    /** TalonFX follower motor controller object */
    private final TalonFX m_followerMotor;

    /** Configuration object for the TalonFX motor */
    private final TalonFXConfiguration m_motorConfig;

    /** Voltage control request object for the TalonFX motor controller */
    private final VoltageOut m_voltReq;

    /** Motion magic voltage control request object for the TalonFX motor controller */
    private MotionMagicVoltage m_request;

    /** Object of the Phoenix Orchestra */
    private Orchestra m_orchestra;
    /** Object of a simulated elevator */
    private final ElevatorSim m_elevatorSim;

    /** Mechanism2d object of an elevator */
    private final Mechanism2d m_mech2d;
    /** MechanismRoot2d object of an elevator */
    private final MechanismRoot2d m_mech2dRoot;
    /** MechanismLigament2d object of an elevator */
    private MechanismLigament2d m_elevatorMech2d;

    /** Object of a system identification routine */
    private final SysIdRoutine m_sysIdRoutine;

    /** Target setpoint for the elevator in meters */
    private double m_setpoint;

    /** Indicates whether the elevator has been homed */
    private boolean m_homed;

    /** Array of songs to be played by the Phoenix Orchestra */
    private String[] m_songs = new String[] {"song1.chrp", "song2.chrp"};

    /** Creates a new ElevatorSubsystem */
    public ElevatorSubsystem() {
        m_leaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);

        /** TalonFX follower motor controller object */
        m_followerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);
    
        /** Configuration object for the TalonFX motor */
        m_motorConfig = new TalonFXConfiguration();
    
        /** Voltage control request object for the TalonFX motor controller */
        m_voltReq = new VoltageOut(0.0);
    
        /** Motion magic voltage control request object for the TalonFX motor controller */
        m_request = new MotionMagicVoltage(0).withSlot(0);
    
        /** Object of the Phoenix Orchestra */
        m_orchestra = new Orchestra();
    
        /** Object of a simulated elevator */
        m_elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2),
                ElevatorConstants.kElevatorGearing, ElevatorConstants.kElevatorCarriageMass,
                ElevatorConstants.kElevatorDrumRadius, ElevatorConstants.kElevatorMinHeightMeters,
                ElevatorConstants.kElevatorMaxHeightMeters, true,
                ElevatorConstants.kElevatorHeightMeters, 0.01, // add some noise
                0);
    
        /** Mechanism2d object of an elevator */
        m_mech2d =
                new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(50));
        /** MechanismRoot2d object of an elevator */
        m_mech2dRoot =
                m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
        /** MechanismLigament2d object of an elevator */
        m_elevatorMech2d =
                m_mech2dRoot.append(new MechanismLigament2d("Elevator",
                        m_elevatorSim.getPositionMeters(), 90, 7, new Color8Bit(Color.kAntiqueWhite)));
    
        if (RobotBase.isSimulation()) {
            //SmartDashboard.putData("Elevator Sim", m_mech2d); commented out for testing
            m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
        }
        /** Object of a system identification routine */
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_leaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null, this));

        // Set motor configuration
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set slot 0 values
        m_motorConfig.Slot0.kP = ElevatorConstants.kElevatorP;
        m_motorConfig.Slot0.kI = ElevatorConstants.kElevatorI;
        m_motorConfig.Slot0.kD = ElevatorConstants.kElevatorD;
        m_motorConfig.Slot0.kS = ElevatorConstants.kElevatorS;
        m_motorConfig.Slot0.kV = ElevatorConstants.kElevatorV;
        m_motorConfig.Slot0.kA = ElevatorConstants.kElevatorA;
        m_motorConfig.Slot0.kG = ElevatorConstants.kElevatorG;

        // Set gravity type
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Set motion magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity =
                ElevatorConstants.kElevatorMaxVelocity;
        m_motorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorMaxAcceleration;
        // m_motorConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600

        // Set safety limits
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                ElevatorConstants.kElevatorMaxHeightMeters
                        / ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Set current limits
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 125;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        // Create a new HardwareLimitSwitchConfigs object
        // HardwareLimitSwitchConfigs limitConfigs = new HardwareLimitSwitchConfigs();

        // Configure the reverse limit switch
        // limitConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        // limitConfigs.ReverseLimitEnable = true;
        // limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        // Apply the configuration to your TalonFX
        // m_elevatorLeaderMotor.getConfigurator().apply(limitConfigs);

        // Set follower
        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), true));

        // Apply configuration to motors
        m_leaderMotor.getConfigurator().apply(m_motorConfig);
        m_followerMotor.getConfigurator().apply(m_motorConfig);

        // Set update frequencies
        m_leaderMotor.getPosition().setUpdateFrequency(50);
        m_leaderMotor.getVelocity().setUpdateFrequency(50);
        m_leaderMotor.getDutyCycle().setUpdateFrequency(50);
        m_leaderMotor.getMotorVoltage().setUpdateFrequency(50);
        m_leaderMotor.getTorqueCurrent().setUpdateFrequency(50);
        // m_elevatorFollowerMotor.optimizeBusUtilization();
        // m_elevatorLeaderMotor.optimizeBusUtilization();

        m_orchestra.addInstrument(m_leaderMotor);
        m_orchestra.addInstrument(m_followerMotor);

        m_homed = false;
    }

    /**
     * Gets the current position of the elevator in rotations
     * 
     * @return The current position of the elevator in rotations
     */
    public double getPosition() {
        return m_leaderMotor.getPosition().getValueAsDouble();
    }

    /**
     * Gets the current position of the elevator in meters
     * 
     * @return The current position of the elevator in meters
     */
    public double getPositionMeters() {
        return getPosition() * ElevatorConstants.kElevatorMetersPerMotorRotation;
    }

    /**
     * Gets the current velocity of the elevator in m/s
     * 
     * @return The current velocity of the elevator in m/s
     */
    public double getVelocityMetersPerSecond() {
        return m_leaderMotor.getVelocity().getValueAsDouble()
                * ElevatorConstants.kElevatorMetersPerMotorRotation;
    }

    /** Zero the elevator */
    public void zero() {
        m_leaderMotor.setPosition(0);
    }

    /**
     * Returns whether the elevator is at the setpoint within the target error range
     * 
     * @return Whether the elevator is at the setpoint
     */
    public boolean atHeight() {
        return Math.abs(getPositionMeters() - m_setpoint) < ElevatorConstants.kElevatorTargetError;
    }

    /** Stop the motors */
    public void stop() {
        m_leaderMotor.stopMotor();
        m_followerMotor.stopMotor();
    }

    /** Update telemetry, including the mechanism visualization */
    public void updateTelemetry() {
        m_elevatorMech2d.setLength(getPositionMeters());
    }

    /**
     * Set the position of the elevator using Motion Magic control
     * 
     * @param position The position in meters
     */
    public void setPosition(double position) {

        if (position > ElevatorConstants.kElevatorMaxHeightMeters) {
            m_setpoint = ElevatorConstants.kElevatorMaxHeightMeters;
        } else if (position < ElevatorConstants.kElevatorMinHeightMeters) {
            m_setpoint = ElevatorConstants.kElevatorMinHeightMeters;
        } else {
            m_setpoint = position;
        }

        double positionRotations = position / ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_request = m_request.withPosition(positionRotations).withSlot(0);
        m_leaderMotor.setControl(m_request);
    }

    /**
     * Returns whether the elevator is at the retract limit
     * 
     * @return Whether the elevator is at the retract limit
     */
    public boolean isAtRetractLimit() {
        return m_leaderMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    /**
     * Returns the current position of the elevator as an enum
     * <ul>
     * <li><strong>CORAL_L1</strong>: Represents the first level of the Coral height
     * (kElevatorCoralLevel1StartHeight)</li>
     * <li><strong>CORAL_L2</strong>: Represents the second level of the Coral height
     * (kElevatorCoralLevel2Height)</li>
     * <li><strong>CORAL_L3</strong>: Represents the third level of the Coral height
     * (kElevatorCoralLevel3Height)</li>
     * <li><strong>CORAL_STATION</strong>: Represents the Coral station height
     * (kElevatorCoralStationHeight)</li>
     * <li><strong>ALGAE_HIGH</strong>: Represents the high Algae position
     * (kElevatorAlgaeHighHeight)</li>
     * <li><strong>ALGAE_LOW</strong>: Represents the low Algae position
     * (kElevatorAlgaeLowHeight)</li>
     * <li><strong>PROCESSOR</strong>: Represents the processor height
     * (kElevatorProcessorHeight)</li>
     * </ul>
     * 
     * @return The current position of the elevator as an enum
     */
    public ElevatorPosition getElevatorEnumPosition() {
        // Match current position to known positions
        double currentPosition = getPositionMeters();

        if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralLevel1StartHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_L1;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralLevel2Height) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_L2;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralLevel3Height) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_L3;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorCoralStationAndProcessorHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.CORAL_STATION_AND_PROCESSOR;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorAlgaeHighHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.ALGAE_HIGH;
        } else if (Math.abs(currentPosition
                - ElevatorConstants.kElevatorAlgaeLowHeight) < ElevatorConstants.kElevatorTargetError
                        * 2) {
            return ElevatorPosition.ALGAE_LOW;
        } else {
            // Return null or a default value if no position matches
            return ElevatorPosition.UNKNOWN;
        }
    }

    @Override
    public void periodic() {
        if (m_leaderMotor.getClosedLoopReference().getValueAsDouble() == 0
                && m_leaderMotor.getPosition().getValueAsDouble() < 0.5) {
            m_leaderMotor.setVoltage(0);

        } else {
            m_leaderMotor.setControl(m_request);

        }

        // Update SmartDashboard
        //SmartDashboard.putNumber("Elevator position (m)", getPositionMeters()); commented out for testing
        //SmartDashboard.putNumber("Elevator setpoint position (m)", commented out for testing
        //        m_leaderMotor.getClosedLoopReference().getValueAsDouble()
        //                * ElevatorConstants.kElevatorMetersPerMotorRotation);
        
        // SmartDashboard.putNumber("Elevator velocity (m/s)",
        // getVelocityMetersPerSecond());
        // SmartDashboard.putNumber("Rotations",
        // m_elevatorLeaderMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Rotations setpoint",
        // m_elevatorLeaderMotor.getClosedLoopReference().getValueAsDouble());
        // SmartDashboard.putBoolean("Homed", m_homed);
        // SmartDashboard.putNumber("Leader stator current",
        // m_elevatorLeaderMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Follower stator current",
        // m_elevatorFollowerMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Leader supply current",
        // m_elevatorLeaderMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Follower supply current",
        // m_elevatorFollowerMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putBoolean("Reverse limit switch", isAtRetractLimit());

        // String elevatorEnumPosition = (getElevatorEnumPosition() != null) ? getElevatorEnumPosition().toString()
        //         : "N/A";
        // SmartDashboard.putString("Curr Position Name", elevatorEnumPosition);

        updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // Use TalonFXSimState for proper simulation integration
        m_elevatorSim.setInputVoltage(m_leaderMotor.getMotorVoltage().getValueAsDouble());
        m_elevatorSim.update(0.020);

        // Update simulated motor position/velocity
        final double positionRot = m_elevatorSim.getPositionMeters()
                / ElevatorConstants.kElevatorMetersPerMotorRotation;
        final double velocityRps = m_elevatorSim.getVelocityMetersPerSecond()
                / ElevatorConstants.kElevatorMetersPerMotorRotation;

        // Use TalonFXSimState for accurate simulation updates
        m_leaderMotor.getSimState().setRawRotorPosition(positionRot);
        m_leaderMotor.getSimState().setRotorVelocity(velocityRps);

        // Update battery simulation
        RoboRioSim.setVInVoltage(BatterySim
                .calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

}
