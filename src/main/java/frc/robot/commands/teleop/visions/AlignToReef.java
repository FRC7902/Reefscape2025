// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;

public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;
  private double robotRotation = 0;
  private Pose2d robotPose;

  private ProfiledPIDController yControllerFarDis;
  private ProfiledPIDController yControllerCloseDis;  

  private CameraInterface m_autoAlignCam;

  private final RobotContainer m_robotContainer;

  private double aprilTagRotation;
  private double aprilTagYaw;

  private ProfiledPIDController yController;

  double distanceToCenterOfAprilTag = 0;
  double initialRobotPosition = 0; 

  Debouncer m_debouncer;
  
  //Command driveRobotOrientedAngularVelocity;
  //Command driveFieldOrientedAnglularVelocity;


  public AlignToReef(RobotContainer m_robotContainer, int triggerType, CameraInterface m_autoAlignCamera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_robotContainer = m_robotContainer;
    this.m_autoAlignCam = m_autoAlignCamera;

    yControllerFarDis = new ProfiledPIDController(VisionConstants.kPYF, VisionConstants.kIYF, VisionConstants.kDYF, VisionConstants.yConstraints); //to tune
    yControllerCloseDis = new ProfiledPIDController(VisionConstants.kPYC, VisionConstants.kIYC, VisionConstants.kDYC, VisionConstants.yConstraints); //to tune
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    robotPose = RobotContainer.m_swerveSubsystem.getPose();

    robotRotation = robotPose.getRotation().getDegrees();

    yControllerFarDis.reset(m_autoAlignCam.getAprilTagYaw());
    yControllerFarDis.setTolerance(VisionConstants.yControllerTolerance);
    yControllerFarDis = new ProfiledPIDController(VisionConstants.kPYF, VisionConstants.kIYF, VisionConstants.kDYF, VisionConstants.yConstraints); //to tune

    yControllerCloseDis.reset(m_autoAlignCam.getAprilTagYaw());
    yControllerCloseDis.setTolerance(VisionConstants.yControllerTolerance);
    yControllerCloseDis = new ProfiledPIDController(VisionConstants.kPYC, VisionConstants.kIYC, VisionConstants.kDYC, VisionConstants.yConstraints); //to tune
    
    if (m_autoAlignCam.getAprilTagYaw() <= 25) {
      yController = yControllerCloseDis;
    }
    else {
      yController = yControllerFarDis;
    }
    initialRobotPosition = RobotContainer.m_swerveSubsystem.getPose().getY();
    //driveRobotOrientedAngularVelocity = RobotContainer.m_swerveSubsystem.driveFieldOriented(m_robotContainer.driveRobotOriented);
    //driveFieldOrientedAnglularVelocity = RobotContainer.m_swerveSubsystem.driveFieldOriented(m_robotContainer.driveAngularVelocity);

    m_debouncer = new Debouncer(VisionConstants.kMaxTimeToWait, Debouncer.DebounceType.kFalling);

    aprilTagRotation = m_autoAlignCam.getAprilTagRotation();
    aprilTagYaw = m_autoAlignCam.getAprilTagYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    m_autoAlignCam.resetTargetDetector();
    m_autoAlignCam.getCameraResults();
    if (m_autoAlignCam.cameraSawTarget()) {
      robotPose = RobotContainer.m_swerveSubsystem.getPose();
      robotRotation = robotPose.getRotation().getRadians();
      System.out.println("HAWK TUAH!");
      //closestAprilTagPose = m_autoAlignCam.poseOfAprilTag;

      yController.setGoal(0);

      double robotDisplacement = (aprilTagYaw - RobotContainer.m_swerveSubsystem.getPose().getY()) + VisionConstants.kAprilTagOffset;
      //var ySpeed = yController.calculate(robotDisplacement);
      var ySpeed = yController.calculate(m_autoAlignCam.getAprilTagYaw());
      if (yControllerFarDis.atGoal() || robotRotation > 0.4363) {
        System.out.println("Y Controller at Goal");
        ySpeed = 0;
      }

      final double yControllerError = yControllerFarDis.getAccumulatedError();

      hawkTuah("Accumulated Y Distance Error", yControllerError);

      //drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

      RobotContainer.m_swerveSubsystem.alignToAprilTag(0, getDriverControllerLeftY(), ySpeed, 0.5);

      //RobotContainer.m_swerveSubsystem.drive(
        //ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
        //swerveController.getTargetSpeeds(yControllerError, yControllerError, yControllerError, robotDisplacement, yControllerError)
        //ChassisSpeeds.fromRobotRelativeSpeeds(getDriverControllerLeftY(), ySpeed, omegaSpeed, robotPose.getRotation()));
    }
    else if (m_debouncer.calculate(m_autoAlignCam.cameraSawTarget())) {
      endCommand = true;
    }
}
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }


  public void hawkTuah(String text, double key) {
    SmartDashboard.putNumber(text, key);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }

  private double getDriverControllerLeftY() {
    return -RobotContainer.m_driverController.getLeftY();
  }
}


