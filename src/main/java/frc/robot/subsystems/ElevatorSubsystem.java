// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {

  // Declare motor controllers
  private final TalonFX m_elevatorLeaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);
  private final TalonFX m_elevatorFollowerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);

  private TalonFXConfiguration m_elevatorConfig = new TalonFXConfiguration();
  
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);


  private final Encoder m_encoder = new Encoder(0, 1);
  private final PWMSparkMax m_motor = new PWMSparkMax(0);

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  private final ExponentialProfile m_profile = new ExponentialProfile(
      ExponentialProfile.Constraints.fromCharacteristics(
          ElevatorConstants.kMaxV,
          ElevatorConstants.kV,
          ElevatorConstants.kA));

  private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

  /** Object of a simualted elevator */
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      DCMotor.getFalcon500(2),
      ElevatorConstants.kGearRatio,
      ElevatorConstants.kCarriageMass,
      ElevatorConstants.kDrumRadius,
      ElevatorConstants.kElevatorMinHeightMeters,
      ElevatorConstants.kElevatorMaxHeightMeters,
      true,
      ElevatorConstants.kElevatorHeightMeters,
      0.01, // add noise
      0
      );

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
      new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 7, new Color8Bit(Color.kAntiqueWhite)));

  /** Creates a new ElevatorSubsystem */
  public ElevatorSubsystem() {
    m_encoder.setDistancePerPulse(ElevatorConstants.kElevatorDistPerPulse);

    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", m_mech2d);
      m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
    }

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

    // // Gains and PID configuration
    // m_elevatorConfig.Slot0.kP = ElevatorConstants.kP;
    // m_elevatorConfig.Slot0.kI = ElevatorConstants.kI;
    // m_elevatorConfig.Slot0.kD = ElevatorConstants.kD;
    // m_elevatorConfig.Slot0.kS = ElevatorConstants.kS;
    // m_elevatorConfig.Slot0.kV = ElevatorConstants.kV;
    // m_elevatorConfig.Slot0.kA = ElevatorConstants.kA;
    // m_elevatorConfig.Slot0.kG = ElevatorConstants.kG;

    // // Motion Magic configuration
    // m_elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 80; // Adjust as
    // needed
    // m_elevatorConfig.MotionMagic.MotionMagicAcceleration = 160; // Adjust as
    // needed
    // m_elevatorConfig.MotionMagic.MotionMagicJerk = 1600; // Adjust as needed

    // m_elevatorLeaderMotor.getConfigurator().apply(m_elevatorConfig);

    // // Set update frequencies
    // m_elevatorLeaderMotor.getPosition().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getVelocity().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getDutyCycle().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getMotorVoltage().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.getTorqueCurrent().setUpdateFrequency(50);
    // m_elevatorLeaderMotor.optimizeBusUtilization();

    // // Initialize elevator state
    // m_homed = false;

  }

  /** Stop motor output */
  public void stop() {
    m_elevatorLeaderMotor.stopMotor();
    m_elevatorFollowerMotor.stopMotor();

    if (RobotBase.isSimulation()) {
      m_motor.set(0);
    }
  }

  /** Reset Exponential profile to begin on current position on enable */
  public void reset() {
    m_setpoint = new ExponentialProfile.State(m_encoder.getDistance(), 0);
  }

  /**
   * Run control to reach and maintain goal
   * 
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    // Create goal state
    var goalState = new ExponentialProfile.State(goal, 0);

    // Calculate next state
    var next = m_profile.calculate(0.020, m_setpoint, goalState);

    // With the setpoint value, run PID like normal
    double pidOutput = m_pidController.calculate(m_encoder.getDistance(), m_setpoint.position);
    double feedforwardOutput = m_elevatorFeedForward.calculateWithVelocities(m_setpoint.velocity, next.velocity);

    // Set motor output
    m_motor.setVoltage(pidOutput + feedforwardOutput);

    // Update setpoint
    m_setpoint = next;
  }

  /** Update telemetry, including the mechanism visualization */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }

  /** Sets the height of the elevator */
  public void setHeight(double setpoint) {
    // deprecated
    // final MotionMagicVoltage request = new MotionMagicVoltage(setpoint)
    // .withSlot(0)
    // .withFeedForward(m_elevatorFeedForward.calculate(setpoint,
    // m_elevatorSim.getVelocityMetersPerSecond()));
    // m_elevatorLeaderMotor.setControl(request.withPosition(setpoint));
  }

  /** Gets the current height of the elevator */
  public double getHeight() {
    return m_elevatorLeaderMotor.getPosition().getValueAsDouble();
  }

  // /**
  // * Returns a command that will execute a quasistatic test in the given
  // * direction
  // *
  // * @param direction The direction (forward or reverse) to run the test in
  // */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return m_sysIdRoutine.quasistatic(direction);
  // }

  // /**
  // * Returns a command that will execute a dynamic test in the given direction
  // *
  // * @param direction The direction (forward or reverse) to run the test in
  // */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return m_sysIdRoutine.dynamic(direction);
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator position", m_encoder.getDistance());
    SmartDashboard.putNumber("Elevator setpoint position", m_setpoint.position);
    // SmartDashboard.putNumber("Elevator Goal Position", );
    SmartDashboard.putNumber("Elevator setpoint velocity", m_setpoint.velocity);
  }

  @Override
  public void simulationPeriodic() {

    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    m_elevatorSim.update(0.020);

    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }
}
