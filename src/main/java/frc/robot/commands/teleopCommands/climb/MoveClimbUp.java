// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.climb;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveClimbUp extends Command {

  private ClimbSubsystem m_climb;
  private RobotContainer m_robotContainer;

  
  public MoveClimbUp(ClimbSubsystem climb, RobotContainer robotContainer) {
    m_climb = climb;
    m_robotContainer = robotContainer;
    addRequirements(getRequirements());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climb.getEncoderDistance() < ClimbConstants.kClimbRaisedPosition) {
      m_climb.reachGoal(0);
    }
    else if (m_climb.getEncoderDistance() >= ClimbConstants.kClimbRaisedPosition) {
      m_climb.stopMotors();
      m_robotContainer.m_operatorController.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.stopSimMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}