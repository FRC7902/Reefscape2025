// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.coral_indexer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AutomaticIntakeCoralCommand extends Command {

    private double m_power;
    private final Timer timer;

    /** Creates a new IntakeCoralCommand. */
    public AutomaticIntakeCoralCommand(double power) {
        addRequirements(RobotContainer.m_indexSubsystem);
        timer = new Timer();
        m_power = power;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // if (RobotContainer.m_climbSubsystem.isFunnelUnlocked()) {
        //     RobotContainer.m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        //     RobotContainer.m_indexSubsystem.setPower(0);
        // } else if (!RobotContainer.m_indexSubsystem.isShallowBeamBroken()
        //         && !RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
        //     RobotContainer.m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        //     RobotContainer.m_indexSubsystem.setPower(m_power);
        // } else if (RobotContainer.m_indexSubsystem.isShallowBeamBroken()
        //         && !RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
        //     RobotContainer.m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        //     RobotContainer.m_indexSubsystem.setPower(m_power);
        // } else if (!RobotContainer.m_indexSubsystem.isShallowBeamBroken()
        //         && RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
        //     RobotContainer.m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.05);
        //     RobotContainer.m_indexSubsystem.setPower(-m_power * 0.25);
        // } else if (RobotContainer.m_indexSubsystem.isShallowBeamBroken()
        //         && RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
        //     RobotContainer.m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.05);
        //     RobotContainer.m_indexSubsystem.stop();
        // }
    if (!RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
        // RobotContainer.m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        RobotContainer.m_indexSubsystem.setPower(m_power * 0.5);
    }

    else if (RobotContainer.m_indexSubsystem.isDeepBeamBroken()) {
        RobotContainer.m_indexSubsystem.stop();
    }

    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_indexSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
