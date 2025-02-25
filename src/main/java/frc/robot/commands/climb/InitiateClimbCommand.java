// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InitiateClimbCommand extends Command {
  /** Creates a new InitiateClimbCommand. */
  public InitiateClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    RobotContainer.m_climbSubsystem.unlockFunnel();
    // TODO: Move climber to 155
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_climbSubsystem.stopFunnelServos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: End command when climb reaches angle (155), and servo reaches unlocked angle
    return false;
  }
}
