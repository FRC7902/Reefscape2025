// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
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
import frc.robot.FirebirdUtils;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // Declare motor controllers
  private final TalonFX m_elevatorLeaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);
  private final TalonFX m_elevatorFollowerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);

  private MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

  private static final FirebirdUtils util = new FirebirdUtils();

  private final ExponentialProfile m_profile = new ExponentialProfile(
      ExponentialProfile.Constraints.fromCharacteristics(
          ElevatorConstants.kMaxV,
          ElevatorConstants.kV,
          ElevatorConstants.kA));

  private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

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

  private final ElevatorFeedforward m_elevatorFeedForward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV,
      ElevatorConstants.kA);

  private final ProfiledPIDController m_pidController = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));

  private final Mechanism2d m_mech2d = new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(50));
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5),
      Units.inchesToMeters(0.5));
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 7,
          new Color8Bit(Color.kAntiqueWhite)));

  /** Creates a new ElevatorSubsystem */
  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", m_mech2d);
      m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
    }

    TalonFXConfiguration config = new TalonFXConfiguration();

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

    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 160;
    config.MotionMagic.MotionMagicJerk = 1600;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = util.metersToRotations(ElevatorConstants.kElevatorMaxHeightMeters);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_elevatorLeaderMotor.getConfigurator().apply(config);
    m_elevatorFollowerMotor.getConfigurator().apply(config);

    m_elevatorFollowerMotor.setControl(new Follower(m_elevatorLeaderMotor.getDeviceID(), true));

    // // Base Motor configuration
    // m_elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // m_elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // m_elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // m_elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    // m_elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    // m_elevatorConfig.CurrentLimits.StatorCurrentLimit = 125;

    // // Follower motor setup
    // m_elevatorFollowerMotor.getConfigurator().apply(m_elevatorConfig);
    // m_elevatorFollowerMotor.optimizeBusUtilization();
    // m_elevatorFollowerMotor.setControl(new
    // Follower(ElevatorConstants.kElevatorLeaderCAN, true));

    // // Leader motor additional configuration
    // m_elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // m_elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 80;
    // m_elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // m_elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.25;

    // // Set update frequencies
    // m_elevatorLeaderMotor.getPosition().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getVelocity().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getDutyCycle().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getMotorVoltage().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getTorqueCurrent().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.optimizeBusUtilization();

  }

  /** Stop the motors */
  public void stop() {
    m_elevatorLeaderMotor.stopMotor();
    m_elevatorFollowerMotor.stopMotor();
  }

  /** Reset Exponential profile to begin on current position on enable */
  public void reset() {
    m_setpoint = new ExponentialProfile.State(m_elevatorLeaderMotor.getPosition().getValueAsDouble(), 0);
  }

  /**
   * Run control to reach and maintain goal
   * 
   * @param goal the position to maintain
   */
  // public void reachGoal(double goal) {
  // // Create goal state
  // var goalState = new ExponentialProfile.State(goal, 0);

  // // Calculate next state
  // var next = m_profile.calculate(0.020, m_setpoint, goalState);

  // // With the setpoint value, run PID like normal
  // double pidOutput =
  // m_pidController.calculate(m_elevatorLeaderMotor.getPosition().getValueAsDouble(),
  // m_setpoint.position);
  // double feedforwardOutput =
  // m_elevatorFeedForward.calculateWithVelocities(m_setpoint.velocity,
  // next.velocity);

  // // Set motor output
  // m_elevatorLeaderMotor.setVoltage(pidOutput + feedforwardOutput);

  // // Update setpoint
  // m_setpoint = next;
  // }

  /** Update telemetry, including the mechanism visualization */
  public void updateTelemetry() {
    m_elevatorMech2d.setLength(getPosition());
  }

  /**
   * Set the position of the elevator
   * 
   * @param position of elevator
   */
  public void setPosition(double position) {
    // Use Motion Magic for smooth trajectory following
    m_elevatorLeaderMotor.setControl(m_request.withPosition(position)
        .withSlot(0));

    // Update internal state
    m_setpoint = new ExponentialProfile.State(position, 0);
  }

  /** Gets the current position of the elevator */
  public double getPosition() {
    return m_elevatorLeaderMotor.getPosition().getValueAsDouble();
  }

  /** Zero the elevator */
  public void zero() {
    setPosition(0);
    m_elevatorLeaderMotor.setPosition(0);
    setSetpoint(0);
  }

  /** Returns whether the elevator is at the setpoint */
  public boolean isFinished() {
    return Math.abs(getPosition() - m_setpoint.position) < ElevatorConstants.kTargetError;
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = new ExponentialProfile.State(setpoint, 0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elevator position", util.rotationsToMeters(getPosition()));
    SmartDashboard.putNumber("Elevator setpoint position", util.rotationsToMeters(m_elevatorLeaderMotor.getClosedLoopReference().getValueAsDouble()));

    SmartDashboard.putNumber("Elevator velocity", m_elevatorLeaderMotor.get());
    SmartDashboard.putNumber("Elevator setpoint velocity", m_setpoint.velocity);

    updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Use TalonFXSimState for proper simulation integration
    m_elevatorSim.setInputVoltage(m_elevatorLeaderMotor.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(0.020);

    // Update simulated motor position/velocity
    final double positionRot = util.metersToRotations(m_elevatorSim.getPositionMeters());
    final double velocityRps = util.metersToRotations(m_elevatorSim.getVelocityMetersPerSecond());

    // Use TalonFXSimState for accurate simulation updates
    m_elevatorLeaderMotor.getSimState().setRawRotorPosition(positionRot);
    m_elevatorLeaderMotor.getSimState().setRotorVelocity(velocityRps);

    // Update battery simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

}