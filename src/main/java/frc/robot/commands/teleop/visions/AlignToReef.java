// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;
import swervelib.SwerveController;

public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController yController;

  private CameraInterface m_autoAlignCam;

  private final RobotContainer m_RobotContainer;

  private final int triggerPressed;

  private double aprilTagOffset;

  private Pose2d aprilTagPose;
  private Pose2d robotPose;

  private Transform2d aprilTagDistance;

  private SwerveController yawController;
  

  public AlignToReef(CameraInterface m_autoAlignCamera, RobotContainer m_RobotContainer, int triggerPressed) {
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
    aprilTagPose = m_autoAlignCam.getNearestAprilTag(robotPose);
    aprilTagDistance = aprilTagPose.minus(robotPose);

    if (triggerPressed == 0) {
      aprilTagOffset = VisionConstants.rightReefToAprilTagOffset;
    }

    else if (triggerPressed == 1) {
      aprilTagOffset = VisionConstants.leftReefToAprilTagOffset;
    }

    // if (Math.abs(aprilTagDistance.getY()) < VisionConstants.kSecondPIDControllerStartingPoint) {
    //   yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune
    // }

    // else {
    //   yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    // }

    yawController = RobotContainer.m_swerveSubsystem.getSwerveDrive().getSwerveController();

    yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune

    
    yController.reset(aprilTagDistance.getY());
    yController.setTolerance(VisionConstants.yControllerTolerance);
    yController.setGoal(0.0);

    m_autoAlignCam.turnLEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {  

    robotPose = RobotContainer.m_swerveSubsystem.getPose();
    aprilTagDistance = aprilTagPose.minus(robotPose);

    var ySpeed = yController.calculate(aprilTagDistance.getY() + aprilTagOffset);
    if (yController.atGoal()) {
      ySpeed = 0;
    }
  

    hawkTuah("Y Error", yController.getPositionError());
    hawkTuah("April Tag Rotation", (aprilTagPose.getRotation().getDegrees()));
    hawkTuah("April Tag Y STUFFERRR", aprilTagDistance.getY());

     double multiplier = Math.round(aprilTagPose.getRotation().getRadians() / Math.abs(aprilTagPose.getRotation().getRadians()));

    double rotation;

    double rotationDifference = Math.abs(aprilTagDistance.getRotation().getRadians());

    boolean hasLargeRotationDifference = Math.abs(aprilTagDistance.getRotation().getRadians()) > Math.toRadians(3);   

    if (aprilTagPose.getRotation().getRadians() == 0) {
      rotation = Math.PI;
    }
    
    else {
      rotation = aprilTagPose.getRotation().getRadians() - (Math.PI * multiplier);
    }
     //double rotation = aprilTagPose.getRotation().unaryMinus().getRadians();

    // hawkTuah("target rotation for tag", Math.toDegrees(rotation));

    RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(rotation, getDriverControllerLeftY(), -ySpeed);


    // if (hasLargeRotationDifference) {
    //   RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(rotation, getDriverControllerLeftY(), 0);
    // }
    // else {
    //   RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(rotation, getDriverControllerLeftY(), -ySpeed);
    // }
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_autoAlignCam.turnLEDOff();
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