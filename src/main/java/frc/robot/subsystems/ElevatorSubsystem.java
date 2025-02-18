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

  // Declare motor controllers
  private final TalonFX m_elevatorLeaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);
  private final TalonFX m_elevatorFollowerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);

  // Declare motor configuration
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  // Declare voltage output
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  // Declare Motion Magic voltage
  private MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

  // Declare orchestra
  private Orchestra m_orchestra = new Orchestra();

  /** Object of a simulated elevator */
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      DCMotor.getFalcon500(2),
      ElevatorConstants.kElevatorGearing,
      ElevatorConstants.kElevatorCarriageMass,
      ElevatorConstants.kElevatorDrumRadius,
      ElevatorConstants.kElevatorMinHeightMeters,
      ElevatorConstants.kElevatorMaxHeightMeters,
      true,
      ElevatorConstants.kElevatorHeightMeters,
      0.01, // add some noise
      0);

  private final Mechanism2d m_mech2d = new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(50));
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5),
      Units.inchesToMeters(0.5));
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 7,
          new Color8Bit(Color.kAntiqueWhite)));

  // System identification routine
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(4),
          null,
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> m_elevatorLeaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
          null,
          this));

  private double m_setpoint;
  private boolean m_homed;

  private String[] m_songs = new String[] {
    "song1.chrp",
    "song2.chrp"
  };

  /** Creates a new ElevatorSubsystem */
  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", m_mech2d);
      m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
    }

    // Set motor configuration
    m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set slot
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
    m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorMaxVelocity; // Target cruise velocity of 80 rps
    m_motorConfig.MotionMagic.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    // m_motorConfig.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // Set safety limits
    m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kElevatorMaxHeightMeters / ElevatorConstants.kElevatorMetersPerMotorRotation;
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
    m_elevatorFollowerMotor.setControl(new Follower(m_elevatorLeaderMotor.getDeviceID(), true));

    // Apply configuration to motors
    m_elevatorLeaderMotor.getConfigurator().apply(m_motorConfig);
    m_elevatorFollowerMotor.getConfigurator().apply(m_motorConfig);

    
    // Set update frequencies
    m_elevatorLeaderMotor.getPosition().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getVelocity().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getDutyCycle().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getMotorVoltage().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getTorqueCurrent().setUpdateFrequency(50);
    // m_elevatorFollowerMotor.optimizeBusUtilization();
    // m_elevatorLeaderMotor.optimizeBusUtilization();

      m_orchestra.addInstrument(m_elevatorLeaderMotor);
    m_orchestra.addInstrument(m_elevatorFollowerMotor);

    m_homed = false;
  }

  /** Gets the current position of the elevator in rotations */
  public double getPosition() {
    return m_elevatorLeaderMotor.getPosition().getValueAsDouble();
  }

  /** Gets the current position of the elevator in meters */
  public double getPositionMeters() {
    return getPosition() * ElevatorConstants.kElevatorMetersPerMotorRotation;
  }

  /** Gets the current velocity of the elevator in m/s */
  public double getVelocityMetersPerSecond() {
    return m_elevatorLeaderMotor.getVelocity().getValueAsDouble() * ElevatorConstants.kElevatorMetersPerMotorRotation;
  }

  /** Zero the elevator */
  public void zero() {
    m_elevatorLeaderMotor.setPosition(0);
  }

  /** Returns whether the elevator is at the setpoint */
  public boolean atHeight() {
    return Math.abs(getPositionMeters() - m_setpoint) < ElevatorConstants.kElevatorTargetError;
  }

  /** Stop the motors */
  public void stop() {
    m_elevatorLeaderMotor.stopMotor();
    m_elevatorFollowerMotor.stopMotor();
  }

  /** Update telemetry, including the mechanism visualization */
  public void updateTelemetry() {
    m_elevatorMech2d.setLength(getPositionMeters());
  }

  /**
   * Set the setpoint of the elevator
   * 
   * @param setpoint The setpoint in meters
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * Set the position of the elevator
   * 
   * @param position The position in meters
   */
  public void setPosition(double position) {
    m_setpoint = position;
    double positionRotations = position / ElevatorConstants.kElevatorMetersPerMotorRotation;
    m_request = m_request.withPosition(positionRotations).withSlot(0);
    m_elevatorLeaderMotor.setControl(m_request);
  }

  /** Returns whether the elevator is at the retract limit */
  public boolean isAtRetractLimit() {
    return m_elevatorLeaderMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  /**
   * Returns a command that will execute a quasistatic test in the given
   * direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    m_elevatorLeaderMotor.setControl(m_request);
   
    // Update SmartDashboard
    SmartDashboard.putNumber("Elevator position (m)", getPositionMeters());
    SmartDashboard.putNumber("Elevator setpoint position (m)", m_elevatorLeaderMotor.getClosedLoopReference().getValueAsDouble() * ElevatorConstants.kElevatorMetersPerMotorRotation);
    // SmartDashboard.putNumber("Elevator velocity (m/s)", getVelocityMetersPerSecond());
    // SmartDashboard.putNumber("Rotations", m_elevatorLeaderMotor.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Rotations setpoint", m_elevatorLeaderMotor.getClosedLoopReference().getValueAsDouble());
    // SmartDashboard.putBoolean("Homed", m_homed);
    // SmartDashboard.putNumber("Leader stator current", m_elevatorLeaderMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Follower stator current", m_elevatorFollowerMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Leader supply current", m_elevatorLeaderMotor.getSupplyCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Follower supply current", m_elevatorFollowerMotor.getSupplyCurrent().getValueAsDouble());
    // SmartDashboard.putBoolean("Reverse limit switch", isAtRetractLimit());

    updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Use TalonFXSimState for proper simulation integration
    m_elevatorSim.setInputVoltage(m_elevatorLeaderMotor.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(0.020);

    // Update simulated motor position/velocity
    final double positionRot = m_elevatorSim.getPositionMeters() / ElevatorConstants.kElevatorMetersPerMotorRotation;
    final double velocityRps = m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.kElevatorMetersPerMotorRotation;

    // Use TalonFXSimState for accurate simulation updates
    m_elevatorLeaderMotor.getSimState().setRawRotorPosition(positionRot);
    m_elevatorLeaderMotor.getSimState().setRotorVelocity(velocityRps);

    // Update battery simulation
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

}
