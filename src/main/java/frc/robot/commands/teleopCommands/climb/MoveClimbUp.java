// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.RobotContainer;

public class MoveClimbUp extends Command {

  public MoveClimbUp() {
    addRequirements(RobotContainer.m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_climbSubsystem.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * if (RobotContainer.m_climbSubsystem.getEncoderDistance() <
     * ClimbConstants.kClimbRaisedPosition) {
     * RobotContainer.m_climbSubsystem.driveMotors(ClimbConstants.
     * kClimbRaisedPosition);
     * }
     * else if (RobotContainer.m_climbSubsystem.getEncoderDistance() >=
     * ClimbConstants.kClimbRaisedPosition)
     * {
     * RobotContainer.m_climbSubsystem.stopMotors();
     * m_robotContainer.m_operatorController.setRumble(RumbleType.kBothRumble, 1);
     * }
     */
    RobotContainer.m_climbSubsystem.setVoltage(ClimbConstants.kMotorVoltageUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_climbSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}