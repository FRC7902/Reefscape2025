// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;
  private double reefOffset = 0;

  public AlignToReef() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  public SwerveInputStream getDriveAngularVelocity() {
    return SwerveInputStream.of(RobotContainer.drivebase.getSwerveDrive(), 
    () -> RobotContainer.m_driverController.getLeftY() * -1,
    () -> RobotContainer.m_driverController.getLeftX() * -1)
    .withControllerRotationAxis(() -> RobotContainer.m_driverController.getRightX() * -1)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);        
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_autoAlignCam.camera.getAllUnreadResults();
    RobotContainer.m_autoAlignCam.resetTargetDetector();
    endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    if (RobotContainer.m_driverController.povRight().getAsBoolean()) {
      reefOffset = 1.3; // to test
    }
    else if (RobotContainer.m_driverController.povLeft().getAsBoolean()) {
      reefOffset = -1.3; // to test
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(RobotContainer.m_autoAlignCam.cameraSawTarget())) {
      RobotContainer.m_autoAlignCam.getCameraResults();
      RobotContainer.drivebase.driveFieldOriented(getDriveAngularVelocity()); 
    }
    else if (RobotContainer.m_autoAlignCam.cameraSawTarget()) {
      DoubleSupplier xTrans = () -> RobotContainer.m_autoAlignCam.poseFromRobotToTag.getX();
      DoubleSupplier yTrans = () -> (RobotContainer.m_autoAlignCam.poseFromRobotToTag.getY() + reefOffset);
      DoubleSupplier maxAngularRotation = () -> RobotContainer.drivebase.getMaximumChassisAngularVelocity();
      RobotContainer.drivebase.driveCommand(xTrans, yTrans, maxAngularRotation);
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
