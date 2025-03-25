// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;

public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController yController;

  private double aprilTagOffset;

  private Pose2d aprilTagPose;
  private Pose2d robotPose;

  private Twist2d aprilTagDistance;
  

  public AlignToReef() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = RobotContainer.m_swerveSubsystem.getPose();
    aprilTagPose = RobotContainer.m_cameraSubsystem.getNearestAprilTag(robotPose);

    if (RobotContainer.m_driverController.rightTrigger(0.05).getAsBoolean()) {
      aprilTagOffset = VisionConstants.rightReefToAprilTagOffset;
    }

    else if (RobotContainer.m_driverController.leftTrigger(0.05).getAsBoolean()) {
      aprilTagOffset = VisionConstants.leftReefToAprilTagOffset;
    }

    // if (Math.abs(aprilTagDistance.getY()) < VisionConstants.kSecondPIDControllerStartingPoint) {
    //   yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune
    // }

    // else {
    //   yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    // }

    yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune


    yController.reset(robotPose.getY());
    yController.setTolerance(VisionConstants.yControllerTolerance);
    yController.setGoal(aprilTagPose.getY() + aprilTagOffset);

    RobotContainer.m_cameraSubsystem.turnLEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {  

    robotPose = RobotContainer.m_swerveSubsystem.getPose();

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      System.out.println("Y Controller at Goal");
      ySpeed = 0;
    }
  

    hawkTuah("Y Error", yController.getPositionError());
    hawkTuah("April Tag Rotation", (aprilTagPose.getRotation().getDegrees()));
    hawkTuah("Distance to April Tag", (aprilTagPose.getY() - robotPose.getY()));

    double multiplier = Math.round(aprilTagPose.getRotation().getRadians() / Math.abs(aprilTagPose.getRotation().getRadians()));

    double rotation;

    if (aprilTagPose.getRotation().getRadians() == 0) {
      rotation = Math.PI;
    }
    
    else {
      rotation = aprilTagPose.getRotation().getRadians() - (Math.PI * multiplier);
    }
     //double rotation = aprilTagPose.getRotation().unaryMinus().getRadians();

    // hawkTuah("target rotation for tag", Math.toDegrees(rotation));

    RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(rotation, getDriverControllerLeftY(), -ySpeed, false);
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_cameraSubsystem.turnLEDOff();
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