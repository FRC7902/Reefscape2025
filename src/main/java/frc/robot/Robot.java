// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(0);
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  public Robot() {
    super(0.020);
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    m_elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevator.simulationPeriodic();
  }

  @Override
  public void teleopInit() {
    m_elevator.reset();
  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.getTrigger()) {
      // Here, we set the constant setpoint
      m_elevator.reachGoal(ElevatorConstants.kLevel2);
    } else {
      // Otherwise, we update the setpoint to 1 meter
      m_elevator.reachGoal(ElevatorConstants.kLevel1);
    }
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
