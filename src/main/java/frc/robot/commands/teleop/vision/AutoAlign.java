// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.visions.ReefSide;
import swervelib.SwerveController;
import swervelib.SwerveInputStream;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AutoAlign extends Command {
    /** Creates a new NullCommand. */
    private final SwerveSubsystem m_swerveSubsystem;
    private final SwerveInputStream driveAngularVelocity;
    private final ReefSide m_reefSide;
    Command autoAlign;

    public AutoAlign(SwerveSubsystem m_swerveSubsystem, SwerveInputStream driveAngularVelocity, ReefSide m_reefSide) {
        this.m_swerveSubsystem = m_swerveSubsystem;
        this.driveAngularVelocity = driveAngularVelocity;
        this.m_reefSide = m_reefSide;
        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_reefSide == ReefSide.LEFT) {
            autoAlign = AutoBuilder.pathfindToPose(m_swerveSubsystem.getNearestLeftWaypoint(), new PathConstraints(10, 10, 10, 10));
        }

        else if (m_reefSide == ReefSide.RIGHT) {
            autoAlign = AutoBuilder.pathfindToPose(m_swerveSubsystem.getNearestRightWaypoint(), new PathConstraints(10, 10, 10, 10));
        }

        else if (m_reefSide == ReefSide.MIDDLE) {
            autoAlign = AutoBuilder.pathfindToPose(m_swerveSubsystem.getNearestMiddleWaypoint(), new PathConstraints(10, 10, 10, 10));
        }

        autoAlign.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        autoAlign.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return autoAlign.isFinished();
    }
}
