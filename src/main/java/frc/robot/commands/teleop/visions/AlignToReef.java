// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralIndexerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.visions.CameraInterface;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new InitiateClimbCommand. */
  private SwerveSubsystem drivebase;
  private CameraInterface autoAlignCam;
  private CommandXboxController m_driverController;
  private RobotContainer m_robotContainer;
  private CoralIndexerSubsystem m_coralIndexerSubsystem;
  private boolean endCommand = false;

  public AlignToReef(SwerveSubsystem drivebase, CameraInterface autoAlignCam, CommandXboxController m_driverController, RobotContainer m_robotContainer, CoralIndexerSubsystem m_coralIndexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
    this.drivebase = drivebase;
    this.autoAlignCam = autoAlignCam;
    this.m_driverController = m_driverController;
    this.m_robotContainer = m_robotContainer;
    this.m_coralIndexerSubsystem = m_coralIndexerSubsystem;
  }

  public SwerveInputStream getDriveAngularVelocity() {
    return SwerveInputStream.of(drivebase.getSwerveDrive(), 
    () -> m_driverController.getLeftY() * -1,
    () -> m_driverController.getLeftX() * -1)
    .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);        
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAlignCam.camera.getAllUnreadResults();
    autoAlignCam.resetTargetDetector();
    endCommand = !m_coralIndexerSubsystem.hasCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(autoAlignCam.cameraSawTarget())) {
      autoAlignCam.getCameraResults();
      Commands.run(() -> drivebase.driveFieldOriented(getDriveAngularVelocity())); 
    }
    else if (autoAlignCam.cameraSawTarget()) {
      DoubleSupplier xTrans = () -> autoAlignCam.poseFromRobotToTag.getX();
      DoubleSupplier yTrans = () -> autoAlignCam.poseFromRobotToTag.getY();
      DoubleSupplier maxAngularRotation = () -> drivebase.getMaximumChassisAngularVelocity();
      Commands.run(() -> drivebase.driveCommand(xTrans, yTrans, maxAngularRotation));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return endCommand;
    return false;
  }
}
