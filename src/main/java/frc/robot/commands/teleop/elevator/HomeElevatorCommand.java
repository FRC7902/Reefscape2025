// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class HomeElevatorCommand extends Command {

  // Define a threshold that represents the current spike when the elevator bottom is reached.
  private final double currentThreshold = 40.0; // Adjust this based on your motor’s
                                                // characteristics.
  // A timeout to ensure safety if the spike never occurs.
  private final Timer timer = new Timer();
  private final double timeoutSeconds = 3.0;

  /** Creates a new HomeElevatorCommand. */
  public HomeElevatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if the current spike has occurred.
    if (RobotContainer.m_elevatorSubsystem.getMotorCurrent() < currentThreshold) {
      // Drive downward at a slow speed.
      // RobotContainer.m_elevatorSubsystem.moveDown(0.2);
      RobotContainer.m_elevatorSubsystem.getLeaderMotor().setVoltage(-1.0);

    } else {
      // Current spike detected: stop the motor and reset the encoder.
      RobotContainer.m_elevatorSubsystem.stop();
      RobotContainer.m_elevatorSubsystem.zero();
      // Optionally, you can log or set a flag that homing is complete.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_elevatorSubsystem.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_elevatorSubsystem.getMotorCurrent() >= currentThreshold
        || timer.get() > timeoutSeconds;
  }
}
