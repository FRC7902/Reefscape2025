// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.visions.CameraInterface;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CheckForAprilTag extends Command {
  /** Creates a new AlignToReefCommand. */
  private CameraInterface m_autoAlignCam;


  public CheckForAprilTag(CameraInterface m_autoAlignCam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
    this.m_autoAlignCam = m_autoAlignCam;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    m_autoAlignCam.resetTargetDetector();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    m_autoAlignCam.getCameraResults();
}
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_autoAlignCam.cameraSawTarget() && m_autoAlignCam.cameraViewingCorrectAprilTag();
  }

}