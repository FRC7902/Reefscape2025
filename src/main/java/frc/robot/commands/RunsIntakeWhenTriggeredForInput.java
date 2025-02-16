// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunsIntakeWhenTriggeredForInput extends Command {
  /** Creates a new RunsIntakeWhenTriggeredForInput. */
  private IndexSubsystem m_index;

  public RunsIntakeWhenTriggeredForInput(IndexSubsystem index) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_index = index;
    addRequirements(m_index); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_index.isBeamBroken()) {
      // Stop the motor if the beam is no longer broken
      m_index.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
