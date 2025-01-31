// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // Declare motor controllers
  private final TalonFX m_elevatorLeader = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);
  private final TalonFX m_elevatorFollower = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);

  private final CANcoder m_encoder = new CANcoder(0);
  // private final Encoder m_encoder = new Encoder(0, 1);

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
    0
  );

  ElevatorFeedforward m_elevatorFeedForward = new ElevatorFeedforward(
    ElevatorConstants.kElevatorkS,
    ElevatorConstants.kElevatorkG,
    ElevatorConstants.kElevatorkV,
    ElevatorConstants.kElevatorkA
  );
  
  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    //m_encoder.setDistancePerPulse(ElevatorConstants.kElevatorDistPerPulse);

    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator", m_mech2d);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {

  }

  public void stop() {
    m_elevatorLeader.set(0);
  }

}

