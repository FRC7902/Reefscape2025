// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;
  private double reefOffset = 0;
  

  public AlignToReef() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  private SwerveInputStream getDriveAngularVelocity() {
    return SwerveInputStream.of(RobotContainer.drivebase.getSwerveDrive(), 
    () -> RobotContainer.m_driverController.getLeftY() * -1,
    () -> RobotContainer.m_driverController.getLeftX() * -1)
    .withControllerRotationAxis(() -> RobotContainer.m_driverController.getRightX() * -1)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);        
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.m_autoAlignCam.clearCameraFIFOBuffer();
    RobotContainer.m_autoAlignCam.resetTargetDetector();
    endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    if (RobotContainer.m_driverController.povRight().getAsBoolean()) {
      reefOffset = VisionConstants.reefToAprilTagOffset; // to measure
    }
    else if (RobotContainer.m_driverController.povLeft().getAsBoolean()) {
      reefOffset = -VisionConstants.reefToAprilTagOffset; // to measure
    }
    RobotContainer.m_autoAlignCam.getCameraResults();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  /*
  @Override 
  public void execute() {
    if (!(RobotContainer.m_autoAlignCam.cameraSawTarget())) {
      RobotContainer.m_autoAlignCam.getCameraResults();
      RobotContainer.drivebase.driveFieldOriented(getDriveAngularVelocity()); 
    }
    else if (RobotContainer.m_autoAlignCam.cameraSawTarget()) {
      System.out.println("HAWK TUAH!");
      final DoubleSupplier xTrans = () -> RobotContainer.m_autoAlignCam.getRobotToTagPose().getX();
      final DoubleSupplier yTrans = () -> (RobotContainer.m_autoAlignCam.getRobotToTagPose().getY());
      final DoubleSupplier maxAngularRotation = () -> RobotContainer.drivebase.getMaximumChassisAngularVelocity();
      //Commands.defer(() -> RobotContainer.drivebase.createPathToAprilTag(RobotContainer.m_autoAlignCam.getRobotToTagPose()), Set.of(RobotContainer.drivebase));
      //RobotContainer.drivebase.drive(new Translation2d(RobotContainer.m_autoAlignCam.getRobotToTagPose().getX(), RobotContainer.m_autoAlignCam.getRobotToTagPose().getY()), RobotContainer.m_autoAlignCam.getYaw(), true);
      RobotContainer.m_autoAlignCam.setTagWayPoint();
      endCommand = true;
    }
    */
  @Override
  public void execute() {
    System.out.println("HAWK TUAH!");
    final DoubleSupplier xTrans = () -> RobotContainer.m_autoAlignCam.getRobotToTagPose().getX();
    final DoubleSupplier yTrans = () -> (RobotContainer.m_autoAlignCam.getRobotToTagPose().getY());
    final DoubleSupplier maxAngularRotation = () -> RobotContainer.drivebase.getMaximumChassisAngularVelocity();
    //Commands.defer(() -> RobotContainer.drivebase.createPathToAprilTag(RobotContainer.m_autoAlignCam.getRobotToTagPose()), Set.of(RobotContainer.drivebase));
    //RobotContainer.drivebase.drive(new Translation2d(RobotContainer.m_autoAlignCam.getRobotToTagPose().getX(), RobotContainer.m_autoAlignCam.getRobotToTagPose().getY()), RobotContainer.m_autoAlignCam.getYaw(), true);
    //RobotContainer.m_autoAlignCam.setTagWayPoint();
    endCommand = true;   
  }
    //


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
