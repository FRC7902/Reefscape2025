// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.PathPlanner;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;
import frc.robot.visions.ReefSide;
import swervelib.SwerveController;

public class AlignToL1 extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController strafeController;
  private CameraInterface m_autoAlignCam;

  private final ReefSide l1Side;
  private double rotationAmount;

  private double targetAngle;

  private Pose2d aprilTagPose;
  private Pose2d currentRobotPose;

  private Pose2d targetPose;
  private PathConstraints constraints;

  private Transform2d aprilTagDistanceToRobot;

  Command alignCommand;
  

  public AlignToL1(CameraInterface m_autoAlignCamera, ReefSide l1Side) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
    this.l1Side = l1Side;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //gets the pose of the robot and the nearest april tag
    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();
    aprilTagPose = m_autoAlignCam.getNearestAprilTag(currentRobotPose);
    
    //finds the distance between the april tag and the robot
    double multiplier = Math.round(aprilTagPose.getRotation().getRadians() / Math.abs(aprilTagPose.getRotation().getRadians()));


    Rotation2d targetRotation = aprilTagPose.getRotation().rotateBy(new Rotation2d(Math.PI));

    // if (aprilTagPose.getRotation().getRadians() == 0) {
    //   targetAngle = Math.PI;
    // }
    
    // else {
    //   targetAngle = aprilTagPose.getRotation().getRadians() - (Math.PI * multiplier);
    // }

    //determines which side the robot should strafe to based on the auto align button the driver pressed (left or right)
    if (l1Side == ReefSide.RIGHT) {
      targetRotation = targetRotation.rotateBy(new Rotation2d(Math.toRadians(-60)));
    }

    else if (l1Side == ReefSide.LEFT) {
      targetRotation = targetRotation.rotateBy(new Rotation2d(Math.toRadians(60)));
    }

    constraints = new PathConstraints(100, 300, 400, 500);

    targetPose = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(Math.toRadians(targetRotation.getDegrees())));

  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {  
    alignCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    alignCommand.schedule();
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

  public void hawkTuah(String text, double key) {
    SmartDashboard.putNumber(text, key);
  }
}