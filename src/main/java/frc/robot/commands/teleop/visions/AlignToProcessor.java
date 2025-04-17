// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;
import frc.robot.visions.ReefSide;
import swervelib.SwerveController;

public class AlignToProcessor extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController strafeController;
  private CameraInterface m_autoAlignCam;

  private Pose2d processorPose;
  private Pose2d currentRobotPose;

  private Optional<Alliance> alliance;

  private Transform2d processorDistanceToRobot;

  private double multiplier;

  public AlignToProcessor(CameraInterface m_autoAlignCamera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //gets the pose of the robot and the nearest april tag
    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();
    alliance = DriverStation.getAlliance();
    
    if (alliance.get() == Alliance.Red) {
      processorPose = m_autoAlignCam.getAprilTagFieldLayout().getTagPose(3).get().toPose2d();
      multiplier = 1.0;
    }

    else if (alliance.get() == Alliance.Blue) {
      processorPose = m_autoAlignCam.getAprilTagFieldLayout().getTagPose(16).get().toPose2d();
      multiplier = -1.0;
    }
    
    //finds the distance between the april tag and the robot
    processorDistanceToRobot = processorPose.minus(currentRobotPose);


    // if (Math.abs(aprilTagDistance.getY()) < VisionConstants.kSecondPIDControllerStartingPoint) {
    //   yController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune
    // }

    // else {
    //   yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    // }

    //pid controller used to control strafe for auto algin
    strafeController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune

    //sets the start point of the pid controller to be the current distance between the tag and the robot
    strafeController.reset(processorDistanceToRobot.getY());

    //adds tolerance to the y controller as having an error of 0 is quite difficult (although in this case, a tolerance of 0 will only yield correct results for auto align)
    strafeController.setTolerance(VisionConstants.yControllerTolerance);
    
    //goal should be 0, which means there is no distance difference between the april tag (plus the reef offset) and the robot, meaning the robot is aligned 
    strafeController.setGoal(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {  

    //updates robot pose and distance to april tag
    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();
    processorDistanceToRobot = processorPose.minus(currentRobotPose);

    //calculates the speed needed to get to the setpoint
    //if it is at the setpoint (meaning 0 distance difference between the april tag and the robot), set the speed to 0 so the robot stops auto aligning
    var ySpeed = strafeController.calculate(processorDistanceToRobot.getY());
    if (strafeController.atGoal()) {
      ySpeed = 0;
    }

    //prints out the strafe error when auto aligning
    hawkTuah("Auto-align Error", strafeController.getPositionError());
    hawkTuah("April Tag Rotation", (processorPose.getRotation().getDegrees()));
    hawkTuah("tagdis", processorPose.minus(currentRobotPose).getY());


    double rotation = processorPose.getRotation().rotateBy(new Rotation2d(Math.PI)).getRadians();


    double rotationDifference = Math.abs(currentRobotPose.getRotation().getRadians() - rotation);
    boolean hasLargeRotationDifference = rotationDifference > Math.toRadians(17);   

    if (hasLargeRotationDifference) {
      RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(rotation, 0, 0, true);
    }
    else if (!hasLargeRotationDifference) {
      RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(rotation, ySpeed * multiplier, getDriverControllerLeftX(), true);
    }
    // RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation, getDriverControllerLeftY(), -ySpeed);

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

  private double getDriverControllerLeftX() {
    return RobotContainer.m_driverController.getLeftX() * VisionConstants.kAutoAlignSpeedMultiplier;
  }

  public void hawkTuah(String text, double key) {
    SmartDashboard.putNumber(text, key);
  }
}