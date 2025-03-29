// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class SetElevatorPositionCommand extends Command {
    private double m_targetHeight;

    /** Creates a new ElevatorSetpoint. */
    public SetElevatorPositionCommand(double targetHeight) {
        m_targetHeight = targetHeight;
        addRequirements(RobotContainer.m_elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // RobotContainer.m_elevatorSubsystem.setPosition(m_targetHeight);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (m_targetHeight == ElevatorConstants.kElevatorCoralLevel1Height || m_targetHeight == ElevatorConstants.kElevatorCoralLevel2Height || m_targetHeight == ElevatorConstants.kElevatorCoralLevel3Height) {
            if (RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
                RobotContainer.m_elevatorSubsystem.setPosition(m_targetHeight);
            }
        } else {
            RobotContainer.m_elevatorSubsystem.setPosition(m_targetHeight);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.m_elevatorSubsystem.atHeight();
    }
}
