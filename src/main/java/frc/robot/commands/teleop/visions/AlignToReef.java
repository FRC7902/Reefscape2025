// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import com.google.flatbuffers.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.visions.CameraInterface;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new InitiateClimbCommand. */
  private SwerveSubsystem drivebase;
  private CameraInterface autoAlignCam;
  private CommandXboxController m_driverController;

  private boolean endCommand = false;
  public AlignToReef(SwerveSubsystem drivebase, CameraInterface autoAlignCam, CommandXboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivebase);
    this.drivebase = drivebase;
    this.autoAlignCam = autoAlignCam;
    this.m_driverController = m_driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAlignCam.resetTargetDetector();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(autoAlignCam.cameraSawTarget())) {
        autoAlignCam.getCameraResults();
    }
    else if (autoAlignCam.cameraSawTarget()) {
        drivebase.drive(new Translation2d(Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2), autoAlignCam.horizontalDistanceToTag), autoAlignCam.targetYaw, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
