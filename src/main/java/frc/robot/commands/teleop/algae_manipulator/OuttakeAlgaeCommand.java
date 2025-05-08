// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.algae_manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeManipulatorConstants;
import frc.robot.RobotContainer;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class OuttakeAlgaeCommand extends Command {
    /** Creates a new OuttakeGroundAlgaeCommand. */
    public OuttakeAlgaeCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_algaeElevatorManipulatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_algaeElevatorManipulatorSubsystem
                .setIntakeVoltage(AlgaeManipulatorConstants.kOuttakeVoltage);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.m_algaeElevatorManipulatorSubsystem
        .setIntakeVoltage(AlgaeManipulatorConstants.kOuttakeVoltage);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_algaeElevatorManipulatorSubsystem.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
