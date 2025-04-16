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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;
import frc.robot.visions.ReefSide;
import swervelib.SwerveController;

public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */

  private ProfiledPIDController strafeController;
  private CameraInterface m_autoAlignCam;

  private final ReefSide reefSide;
  private double reefToAprilTagOffset;

  private Pose2d aprilTagPose;
  private Pose2d currentRobotPose;

  private Transform2d aprilTagDistanceToRobot;

  public AlignToReef(CameraInterface m_autoAlignCamera, ReefSide reefSide) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
    this.reefSide = reefSide;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // gets the pose of the robot and the nearest april tag
    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();
    aprilTagPose = m_autoAlignCam.getNearestAprilTag(currentRobotPose);

    // finds the distance between the april tag and the robot
    aprilTagDistanceToRobot = aprilTagPose.minus(currentRobotPose);

    // determines which side the robot should strafe to based on the auto align
    // button the driver pressed (left or right)
    if (reefSide == ReefSide.RIGHT) {
      reefToAprilTagOffset = VisionConstants.rightReefToAprilTagOffset;
    }

    else if (reefSide == ReefSide.LEFT) {
      reefToAprilTagOffset = VisionConstants.leftReefToAprilTagOffset;
    }

    // if (Math.abs(aprilTagDistance.getY()) <
    // VisionConstants.kSecondPIDControllerStartingPoint) {
    // yController = new ProfiledPIDController(VisionConstants.kPY2,
    // VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints);
    // //to tune
    // }

    // else {
    // yController = new ProfiledPIDController(VisionConstants.kPY,
    // VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to
    // tune
    // }

    // pid controller used to control strafe for auto algin
    strafeController = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2,
        VisionConstants.yConstraints); // to tune

    // sets the start point of the pid controller to be the current distance between
    // the tag and the robot
    strafeController.reset(aprilTagDistanceToRobot.getY());

    // adds tolerance to the y controller as having an error of 0 is quite difficult
    // (although in this case, a tolerance of 0 will only yield correct results for
    // auto align)
    strafeController.setTolerance(VisionConstants.yControllerTolerance);

    // goal should be 0, which means there is no distance difference between the
    // april tag (plus the reef offset) and the robot, meaning the robot is aligned
    strafeController.setGoal(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    // updates robot pose and distance to april tag
    currentRobotPose = RobotContainer.m_swerveSubsystem.getPose();
    aprilTagDistanceToRobot = aprilTagPose.minus(currentRobotPose);

    // calculates the speed needed to get to the setpoint
    // if it is at the setpoint (meaning 0 distance difference between the april tag
    // and the robot), set the speed to 0 so the robot stops auto aligning
    var ySpeed = strafeController.calculate(aprilTagDistanceToRobot.getY() + reefToAprilTagOffset);
    if (strafeController.atGoal()) {
      ySpeed = 0;
    }

    // prints out the strafe error when auto aligning
    // hawkTuah("Auto-align Error", strafeController.getPositionError());
    // hawkTuah("April Tag Rotation", (aprilTagPose.getRotation().getDegrees()));

    double multiplier = Math
        .round(aprilTagPose.getRotation().getRadians() / Math.abs(aprilTagPose.getRotation().getRadians()));
    double aprilTagRotation;

    if (aprilTagPose.getRotation().getRadians() == 0) {
      aprilTagRotation = Math.PI;
    }

    else {
      aprilTagRotation = aprilTagPose.getRotation().getRadians() - (Math.PI * multiplier);
    }

    double rotationDifference = Math.abs(currentRobotPose.getRotation().getRadians() - aprilTagRotation);
    boolean hasLargeRotationDifference = rotationDifference > Math.toRadians(17);

    // if (hasLargeRotationDifference) {
    // RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation, 0,
    // 0);
    // }
    // else if (!hasLargeRotationDifference) {
    // RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation,
    // getDriverControllerLeftY(), -ySpeed);
    // }
    RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation,
        getDriverControllerLeftY() * DriveConstants.kAutoAlignForwardBackwardSpeedMultiplier, -ySpeed);

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