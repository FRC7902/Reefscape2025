// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.coral_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualIntakeCoralCommand extends Command {
  private double m_power;

    /** Creates a new IntakeCoralCommand. */
    public ManualIntakeCoralCommand(double power) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_indexSubsystem);
        m_power = power;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_indexSubsystem.setPower(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_indexSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return RobotContainer.m_indexSubsystem.hasCoral();
    return RobotContainer.m_indexSubsystem.isBeamBroken();
  }
}
