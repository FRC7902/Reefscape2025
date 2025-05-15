// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimbSubsystem;

public class ManualClimb extends Command {

    private double direction = 0;
    private ClimbSubsystem m_climbSubsystem;
    
    public ManualClimb(ClimbSubsystem m_climbSubsystem, double direction) {
        addRequirements(Robot.m_climbSubsystem);
        this.m_climbSubsystem = m_climbSubsystem;
        this.direction = direction;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climbSubsystem.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climbSubsystem.driveMotors(12 * direction);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;    
    }
}
