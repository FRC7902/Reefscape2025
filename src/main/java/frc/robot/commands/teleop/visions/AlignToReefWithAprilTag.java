// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;

public class AlignToReefWithAprilTag extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController yController;

  private CameraInterface m_autoAlignCam;

  private final RobotContainer m_RobotContainer;

  private final int triggerPressed;

  private double aprilTagOffset = 0;

  private Pose2d robotPose;

  private double targetRotation = 0;
  

  public AlignToReefWithAprilTag(CameraInterface m_autoAlignCamera, RobotContainer m_RobotContainer, int triggerPressed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
    this.m_RobotContainer = m_RobotContainer;
    this.triggerPressed = triggerPressed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = RobotContainer.m_swerveSubsystem.getPose();
    targetRotation = m_autoAlignCam.getNearestAprilTag(robotPose).getRotation().getRadians();

    if (triggerPressed == 0) {
      aprilTagOffset = VisionConstants.kAprilTagOffset;
    }

    else if (triggerPressed == 1) {
      aprilTagOffset = -VisionConstants.kAprilTagOffset;
    }

    if (Math.abs(m_autoAlignCam.getHorizontalDisToTag()) < VisionConstants.kSecondPIDControllerStartingPoint) {
      yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune
    }

    else {
      yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    }

    yController.reset(m_autoAlignCam.getHorizontalDisToTag());
    yController.setTolerance(VisionConstants.yControllerTolerance);
    yController.setGoal(aprilTagOffset);

    m_autoAlignCam.turnLEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {

    var ySpeed = yController.calculate(m_autoAlignCam.getHorizontalDisToTag());
    if (yController.atGoal()) {
      System.out.println("Y Controller at Goal");
      ySpeed = 0;
    }
    
    SmartDashboard.putNumber("Y Error", yController.getPositionError());
    RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(targetRotation, getDriverControllerLeftY(), ySpeed);
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_autoAlignCam.turnLEDOff();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yController.atGoal();
  }

  private double getDriverControllerLeftY() {
    return -RobotContainer.m_driverController.getLeftY();
  }

}