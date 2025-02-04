// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveClimbDown extends Command {

  private ClimbSubsystem m_climb;

  public MoveClimbDown(ClimbSubsystem climb) {
    m_climb = climb;
    addRequirements(getRequirements());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climb.getEncoderDistance() < ClimbConstants.kClimbEncoderLimit) {
        m_climb.setMotorPower(ClimbConstants.kClimbDownMotorSpeed);
    }
    else if (m_climb.getEncoderDistance() >= ClimbConstants.kClimbEncoderLimit) {
        m_climb.stopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}