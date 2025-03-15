// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;

public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;

  private ProfiledPIDController yController;
  private ProfiledPIDController y2Controller;

  private CameraInterface m_autoAlignCam;
  private double aprilTagRotation;

  private double aprilTagID;

  private Transform2d robotToTag;

  private final RobotContainer m_RobotContainer;

  private Pose2d closestTagPose;

  private Pose2d robotPose;

  private double targetRotation;
  

  public AlignToReef(CameraInterface m_autoAlignCamera, RobotContainer m_RobotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
    this.m_RobotContainer = m_RobotContainer;
    System.out.println("hawk tuerrererererere");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    System.out.println("what the HAWWWWWWWWWWWWWWWWWWWWWWWK");


    var visionEst = RobotContainer.rightCamera.getEstimatedGlobalPose();

    visionEst.ifPresent(
      est -> {
        var estStdDevs = RobotContainer.rightCamera.getEstimationStdDevs();
        RobotContainer.m_swerveSubsystem.getSwerveDrive().addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
    );

    var visionEst2 = RobotContainer.leftCamera.getEstimatedGlobalPose();

    visionEst2.ifPresent(
      est -> {
        var estStdDevs2 = RobotContainer.rightCamera.getEstimationStdDevs();
        RobotContainer.m_swerveSubsystem.getSwerveDrive().addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs2);
      }
    );
    
    //List<Pose2d> tagsDetected = RobotContainer.rightCamera.getDetectedPoses();
    //List<Pose2d> tagsDetectedLeft = RobotContainer.leftCamera.getDetectedPoses();

    //tagsDetected.addAll(tagsDetectedLeft);
    
    robotPose = RobotContainer.m_swerveSubsystem.getPose();

    //closestTagPose = robotPose.nearest(tagsDetected);

    closestTagPose = RobotContainer.rightCamera.getClosestPose(robotPose);

    targetRotation = closestTagPose.getRotation().getRadians();

    yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune

    
    yController.reset(robotPose.getY());
    yController.setTolerance(VisionConstants.yControllerTolerance);
    yController.setGoal(closestTagPose.getY() + VisionConstants.kAprilTagOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    robotPose = RobotContainer.m_swerveSubsystem.getPose();
    var ySpeed = yController.calculate(closestTagPose.getY());
    if (yController.atGoal()) {
      System.out.println("Y Controller at Goal");
      ySpeed = 0;
    }
    hawkTuah("Accumulated Y Error", yController.getAccumulatedError());

    System.out.println(closestTagPose.getY());

    RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(targetRotation, getDriverControllerLeftY(), ySpeed, 0.1);
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

  private double getDriverControllerLeftY() {
    return -RobotContainer.m_driverController.getLeftY();
  }

  public void hawkTuah(String text, double key) {
    SmartDashboard.putNumber(text, key);
  }
}