// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.coral_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class OuttakeCoralCommand extends Command {

    private double m_power;

    public OuttakeCoralCommand() {
        addRequirements(Robot.m_indexSubsystem);
        m_power = Constants.CoralIndexerConstants.kOuttakePower;
    }

    public OuttakeCoralCommand(double power) {
        addRequirements(Robot.m_indexSubsystem);
        m_power = power;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.m_indexSubsystem.setPower(m_power);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.m_indexSubsystem.setPower(m_power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.m_indexSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
