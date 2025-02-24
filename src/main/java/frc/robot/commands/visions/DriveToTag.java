package frc.robot.commands.visions;

import java.awt.geom.QuadCurve2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToTag extends Command {

    private CameraSubsystem m_cameraSubsystem;
    private SwerveSubsystem m_swerveSubsystem;
    private int buttonPressed = 0;

    public DriveToTag(CameraSubsystem m_cameraSubsystem, SwerveSubsystem m_swerveSubsystem, int buttonPressed) {
        addRequirements(RobotContainer.m_cameraSubsystem);
        this.m_cameraSubsystem = m_cameraSubsystem;
        this.m_swerveSubsystem = m_swerveSubsystem;
        this.buttonPressed = buttonPressed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d tagPose = m_cameraSubsystem.getAprilTagPose();
        Pose2d robotPos = m_swerveSubsystem.getPose();
        Pose2d moveToTag = null;
        double xTranslation = 0;
        double yTranslation = 0;
        double feta = 0;
        if (tagPose != null) {
            xTranslation = (tagPose.getX() - robotPos.getX());
            feta = tagPose.getRotation().getRadians() - robotPos.getRotation().getRadians();
            if (buttonPressed == 0) {
                yTranslation = (tagPose.getY() - robotPos.getY()) - VisionConstants.kTagMiddleToTagEdge;
            }
            else if (buttonPressed == 1) {
                yTranslation = (tagPose.getY() - robotPos.getY()) + VisionConstants.kTagMiddleToTagEdge;
            }
            moveToTag = new Pose2d(xTranslation, yTranslation, new Rotation2d(feta));
            
            m_swerveSubsystem.driveToPose(moveToTag);

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
