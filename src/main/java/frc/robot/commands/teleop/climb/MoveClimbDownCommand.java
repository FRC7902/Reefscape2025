// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class MoveClimbDownCommand extends Command {

    private ClimbSubsystem m_climbSubsystem;
    
    public MoveClimbDownCommand(ClimbSubsystem m_climbSubsystem) {
        addRequirements(RobotContainer.m_climbSubsystem);
        this.m_climbSubsystem = m_climbSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climbSubsystem.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //m_climbSubsystem.runToAngle(m_operatorController, m_climbSubsystem.getClimbArmAngle(), ClimbConstants.kClimbBackwardLimit, -1);
        m_climbSubsystem.driveMotors(12);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_climbSubsystem.getClimbArmAngle() >= 319;
    }
}
