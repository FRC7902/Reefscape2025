// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
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

  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

  /** Object of a simulated elevator */
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      DCMotor.getFalcon500(2),
      ElevatorConstants.kGearRatio,
      ElevatorConstants.kCarriageMass,
      ElevatorConstants.kDrumRadius,
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

  /** Creates a new ElevatorSubsystem */
  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", m_mech2d);
      m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
    }

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set motor configuration
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set slot
    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kS = ElevatorConstants.kS;
    config.Slot0.kV = ElevatorConstants.kV;
    config.Slot0.kA = ElevatorConstants.kA;
    config.Slot0.kG = ElevatorConstants.kG;

    // Set motion magic
    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 160;
    config.MotionMagic.MotionMagicJerk = 1600;

    // Set safety limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kElevatorMaxHeightMeters
        / ElevatorConstants.kElevatorMetersPerMotorRotation;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    // Set current limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 125;

    // Apply configuration to motors
    m_elevatorLeaderMotor.getConfigurator().apply(config);
    m_elevatorFollowerMotor.getConfigurator().apply(config);

    // Set follower
    m_elevatorFollowerMotor.setControl(new Follower(m_elevatorLeaderMotor.getDeviceID(), true));

    // Set update frequencies
    m_elevatorLeaderMotor.getPosition().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getVelocity().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getDutyCycle().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getMotorVoltage().setUpdateFrequency(50);
    m_elevatorLeaderMotor.getTorqueCurrent().setUpdateFrequency(50);
    m_elevatorFollowerMotor.optimizeBusUtilization();
    m_elevatorLeaderMotor.optimizeBusUtilization();

    m_homed = false;
  }

  /** Stop the motors */
  public void stop() {
    m_elevatorLeaderMotor.stopMotor();
    m_elevatorFollowerMotor.stopMotor();
  }

  /** Update telemetry, including the mechanism visualization */
  public void updateTelemetry() {
    m_elevatorMech2d.setLength(getPosition());
  }

  /**
   * Set the position of the elevator
   *
   * @param position of elevator
   */
  public void setPosition(double positionMeters) {
    double positionRotations = positionMeters / ElevatorConstants.kElevatorMetersPerMotorRotation;
    m_elevatorLeaderMotor.setControl(m_request.withPosition(positionRotations).withSlot(0));
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
    setPosition(0);
    m_elevatorLeaderMotor.setPosition(0);
    setSetpoint(0);
  }

  /** Returns whether the elevator is at the setpoint */
  public boolean atHeight() {
    return Math.abs(getPositionMeters() - m_setpoint) < ElevatorConstants.kTargetError;
  }

  /** Set the setpoint of the elevator */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
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

    if (!atHeight() && m_homed) {
      m_elevatorLeaderMotor.setControl(m_request);
    } else {
      stop();
    }

    // Update SmartDashboard
    SmartDashboard.putNumber("Elevator position", getPositionMeters());
    SmartDashboard.putNumber("Elevator setpoint position", m_elevatorLeaderMotor.getClosedLoopReference().getValueAsDouble() * ElevatorConstants.kElevatorMetersPerMotorRotation);
    SmartDashboard.putNumber("Elevator velocity", getVelocityMetersPerSecond());

    updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Use TalonFXSimState for proper simulation integration
    m_elevatorSim.setInputVoltage(m_elevatorLeaderMotor.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(0.020);

    // Update simulated motor position/velocity
    final double positionRot = m_elevatorSim.getPositionMeters() * ElevatorConstants.kElevatorMetersPerMotorRotation;
    final double velocityRps = m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.kElevatorMetersPerMotorRotation;

    // Use TalonFXSimState for accurate simulation updates
    m_elevatorLeaderMotor.getSimState().setRawRotorPosition(positionRot);
    m_elevatorLeaderMotor.getSimState().setRotorVelocity(velocityRps);

    // Update battery simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

}
