// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import org.photonvision.PhotonUtils;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.visions.CameraInterface;
import swervelib.SwerveController;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;
  private double reefOffset = 0;
  private double robotRotation = 0;
  private Pose2d robotPose;

  private ProfiledPIDController yController;
  private ProfiledPIDController y2Controller;
  private ProfiledPIDController ySelectedController;

  private ProfiledPIDController omegaController;

  private CameraInterface m_autoAlignCam;

  private final RobotContainer m_robotContainer;

  private double aprilTagRotation;

  //double distanceToCenterOfAprilTag = 0;
  //double initialRobotPosition = 0;
  

  public AlignToReef(RobotContainer m_robotContainer, CameraInterface m_autoAlignCamera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_robotContainer = m_robotContainer;
    this.m_autoAlignCam = m_autoAlignCamera;
    yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    y2Controller = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    //RobotContainer.m_autoAlignCam.clearCameraFIFOBuffer();
    robotPose = RobotContainer.m_swerveSubsystem.getPose();

    robotRotation = robotPose.getRotation().getDegrees();

    yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    y2Controller = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune

    if ((m_autoAlignCam.getAprilTagYaw() < 10 && m_autoAlignCam.getAprilTagYaw() > 0) || (m_autoAlignCam.getAprilTagYaw() >= -10 && m_autoAlignCam.getAprilTagYaw() < 0)) {
      yController = y2Controller;
    }

    yController.reset(m_autoAlignCam.getAprilTagYaw());
    yController.setTolerance(VisionConstants.yControllerTolerance);

    //distanceToCenterOfAprilTag = m_autoAlignCam.getAprilTagYaw();
    //initialRobotPosition = RobotContainer.m_swerveSubsystem.getPose().getY();

    //driveRobotOrientedAngularVelocity = RobotContainer.m_swerveSubsystem.driveFieldOriented(m_robotContainer.driveRobotOriented);
    //driveFieldOrientedAnglularVelocity = RobotContainer.m_swerveSubsystem.driveFieldOriented(m_robotContainer.driveAngularVelocity);

    aprilTagRotation = m_autoAlignCam.getAprilTagRotation();

    yController.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    m_autoAlignCam.getCameraResults();
    robotPose = RobotContainer.m_swerveSubsystem.getPose();
    robotRotation = robotPose.getRotation().getRadians();
    System.out.println("HAWK TUAH!");
    //closestAprilTagPose = m_autoAlignCam.poseOfAprilTag;

    //double robotDisplacement = RobotContainer.m_swerveSubsystem.getPose().getY() - initialRobotPosition;
    //var ySpeed = yController.calculate(robotDisplacement);
    var ySpeed = yController.calculate(m_autoAlignCam.getAprilTagYaw());
    if (yController.atGoal()) {
      System.out.println("Y Controller at Goal");
      ySpeed = 0;
    }

    double yControllerError = yController.getAccumulatedError();

    hawkTuah("Accumulated Y Error", yControllerError);

    //drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

    RobotContainer.m_swerveSubsystem.snapToAprilTagAngle(m_autoAlignCam.getAprilTagRotation(), getDriverControllerLeftY(), ySpeed, 0.5);

    //RobotContainer.m_swerveSubsystem.drive(
      //ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
      //swerveController.getTargetSpeeds(yControllerError, yControllerError, yControllerError, robotDisplacement, yControllerError)
      //ChassisSpeeds.fromRobotRelativeSpeeds(getDriverControllerLeftY(), ySpeed, omegaSpeed, robotPose.getRotation()));
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
    //return endCommand;
    return false;
  }

  private double getDriverControllerLeftY() {
    return -RobotContainer.m_driverController.getLeftY();
  }
}