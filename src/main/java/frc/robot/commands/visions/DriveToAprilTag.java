package frc.robot.commands.visions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToAprilTag extends Command {

    private final CameraSubsystem m_cameraSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    public DriveToAprilTag(CameraSubsystem m_cameraSubsystem, SwerveSubsystem m_swerveSubsystem) {
        addRequirements(RobotContainer.m_cameraSubsystem);
        this.m_cameraSubsystem = m_cameraSubsystem;
        this.m_swerveSubsystem = m_swerveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Rotation2d targetRotation = null;
        Translation2d targetTranslation = null;
        if (m_cameraSubsystem.targetDetected) {
            targetRotation = new Rotation2d(m_cameraSubsystem.targetYaw);
            targetTranslation = new Translation2d(m_cameraSubsystem.targetDistance, targetRotation);
            m_swerveSubsystem.drive(targetTranslation, m_cameraSubsystem.targetYaw, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
