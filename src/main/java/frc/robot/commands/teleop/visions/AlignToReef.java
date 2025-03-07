// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


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
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;
  private double reefOffset = 0;
  private Pose2d closestAprilTagPose;
  private Pose2d robotPose;

  private final ProfiledPIDController yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
  private final ProfiledPIDController omegaController = new ProfiledPIDController(VisionConstants.kPOmega, VisionConstants.kIOmega, VisionConstants.kDOmega, VisionConstants.omegaConstraints); //to tune

  private final CameraInterface m_autoAlignCam;
  private final CoralIndexerSubsystem m_indexSubsystem;
  private final SwerveSubsystem drivebase;
  private final CommandXboxController m_driverController;

  public AlignToReef(SwerveSubsystem drivebase, CameraInterface m_autoAlignCam, CoralIndexerSubsystem m_indexSubsystem, CommandXboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivebase, RobotContainer.m_autoAlignCam);
    this.m_autoAlignCam = m_autoAlignCam;
    this.drivebase = drivebase;
    this.m_indexSubsystem = m_indexSubsystem;
    this.m_driverController = m_driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommand = !m_indexSubsystem.hasCoral();
    //RobotContainer.m_autoAlignCam.clearCameraFIFOBuffer();
    m_autoAlignCam.resetTargetDetector();
    robotPose = drivebase.getPose();

    yController.reset(robotPose.getY());
    omegaController.reset(robotPose.getRotation().getRadians());

    yController.setTolerance(VisionConstants.yControllerTolerance);

    if (driverPressedPOVLeft()) {
      reefOffset = VisionConstants.reefToAprilTagOffset; // to measure
    }
    else if (driverPressedPOVRight()) {
      reefOffset = -VisionConstants.reefToAprilTagOffset; // to measure
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    robotPose = drivebase.getPose();
    final boolean cameraSawTarget = m_autoAlignCam.cameraSawTarget();
    if (!cameraSawTarget) {
      m_autoAlignCam.getCameraResults();
      drivebase.driveFieldOriented(getDriveAngularVelocity()); 
    }
    else if (cameraSawTarget) {
      //System.out.println("HAWK TUAH!");
      closestAprilTagPose = m_autoAlignCam.poseOfAprilTag;
      yController.setGoal(closestAprilTagPose.getY() + reefOffset);
      omegaController.setGoal(closestAprilTagPose.getRotation().getRadians());

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        System.out.println("Y Controller at Goal");
        ySpeed = 0;
      }

      //var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      var omegaSpeed = 0;
      if (omegaController.atGoal()) {
        System.out.println("Omega at goal");
        omegaSpeed = 0;
      }

      final double yControllerError = yController.getAccumulatedError();

      SmartDashboard.putNumber("Accumulated Y Error", yControllerError);
      drivebase.drive(
        //ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
        ChassisSpeeds.fromFieldRelativeSpeeds(getDriverControllerLeftY(), ySpeed, omegaSpeed, robotPose.getRotation()));
    }
    
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
    return m_driverController.getLeftY();
  }

  private boolean driverPressedPOVLeft() {
    return m_driverController.povLeft().getAsBoolean();
  }

  private boolean driverPressedPOVRight() {
    return m_driverController.povRight().getAsBoolean();
  }

  private SwerveInputStream getDriveAngularVelocity() {
    return SwerveInputStream.of(drivebase.getSwerveDrive(), 
    () -> m_driverController.getLeftY() * -1,
    () -> m_driverController.getLeftX() * -1)
    .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);        
  }

}


