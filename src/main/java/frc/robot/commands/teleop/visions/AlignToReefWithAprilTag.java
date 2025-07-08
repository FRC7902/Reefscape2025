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
import frc.robot.visions.ReefSide;

public class AlignToReefWithAprilTag extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController yController;

  private CameraInterface m_autoAlignCam;

  private final ReefSide triggerPressed;

  private double aprilTagOffset = 0;

  private double aprilTagDistanceToRobot = 0;

  private int fidicualID = 0;

  private Pose2d currentRobotPose;
  private Pose2d aprilTagPose;  

  public AlignToReefWithAprilTag(CameraInterface m_autoAlignCamera, ReefSide triggerPressed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
    this.triggerPressed = triggerPressed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    fidicualID = m_autoAlignCam.getAprilTagID();

    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();
    aprilTagPose = m_autoAlignCam.getAprilTagFieldLayout().getTagPose(fidicualID).get().toPose2d();

    if (triggerPressed == ReefSide.RIGHT) {
      aprilTagOffset = VisionConstants.rightReefToAprilTagOffset;
    }

    else if (triggerPressed == ReefSide.LEFT) {
      aprilTagOffset = VisionConstants.leftReefToAprilTagOffset;
    }

    aprilTagDistanceToRobot = m_autoAlignCam.getAprilTagDistanceToRobot();

    // if (Math.abs(m_autoAlignCam.getHorizontalDisToTag()) < VisionConstants.kSecondPIDControllerStartingPoint) {
    //   yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune
    // }

    // else {
    //   yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    // }

    yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune

    yController.reset(currentRobotPose.getY() + aprilTagDistanceToRobot);
    yController.setTolerance(VisionConstants.yControllerTolerance);
    yController.setGoal(aprilTagPose.getY() + aprilTagOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {

    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();

    var ySpeed = yController.calculate(currentRobotPose.getY() + aprilTagDistanceToRobot);
    if (yController.atGoal()) {
      ySpeed = 0;
    }
    
    double multiplier = Math.round(aprilTagPose.getRotation().getRadians() / Math.abs(aprilTagPose.getRotation().getRadians()));
    double aprilTagRotation;

    if (aprilTagPose.getRotation().getRadians() == 0) {
      aprilTagRotation = Math.PI;
    }
    
    else {
      aprilTagRotation = aprilTagPose.getRotation().getRadians() - (Math.PI * multiplier);
    }

    double rotationDifference = Math.abs(currentRobotPose.getRotation().getRadians() - aprilTagRotation);
    boolean hasLargeRotationDifference = rotationDifference > Math.toRadians(17);   

    if (hasLargeRotationDifference) {
      RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation, 0, 0);
    }
    else if (!hasLargeRotationDifference) {
      RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation, getDriverControllerLeftY(), -ySpeed);
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

  private double getDriverControllerLeftY() {
    return -RobotContainer.m_driverController.getLeftY();
  }

  public void hawkTuah(String text, double key) {
    SmartDashboard.putNumber(text, key);
  }
}