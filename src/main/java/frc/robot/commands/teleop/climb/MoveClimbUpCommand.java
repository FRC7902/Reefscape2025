// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveClimbUpCommand extends Command {


    public MoveClimbUpCommand() {
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
        // RobotContainer.m_climbSubsystem.runToAngle(m_operatorController,
        // RobotContainer.m_climbSubsystem.getClimbArmAngle(),
        // ClimbConstants.kClimbForwardLimit, 1);
        RobotContainer.m_climbSubsystem.driveMotors(-12);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_climbSubsystem.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return
        // RobotContainer.m_climbSubsystem.isAtTargetAngle(RobotContainer.m_climbSubsystem.getClimbArmAngle(),
        // ClimbConstants.kClimbForwardLimit, 1);
        return RobotContainer.m_climbSubsystem.getClimbArmAngle() <= 90;
    }
}
