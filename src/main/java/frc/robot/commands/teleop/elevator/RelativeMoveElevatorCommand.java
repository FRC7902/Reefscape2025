// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class RelativeMoveElevatorCommand extends Command {

  private double m_meters;

  /** Creates a new RaiseElevatorCommand. */
  public RelativeMoveElevatorCommand(double meters) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_elevatorSubsystem);
    m_meters = meters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currPosition = Robot.m_elevatorSubsystem.getPositionMeters();
    Robot.m_elevatorSubsystem.setPosition(currPosition + m_meters);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
