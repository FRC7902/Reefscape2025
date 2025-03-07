// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;
  private double reefOffset = 0;
  private Pose2d closestAprilTagPose;
  private Pose2d robotPose;
  
  private final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
  private final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
  private final TrapezoidProfile.Constraints omegaConstraints =   new TrapezoidProfile.Constraints(3, 2);

  private final ProfiledPIDController xController = new ProfiledPIDController(0.03, 0.1, 0.1, xConstraints); //to tune
  private final ProfiledPIDController yController = new ProfiledPIDController(0.03, 0.1, 0.1, yConstraints); //to tune
  private final ProfiledPIDController omegaController = new ProfiledPIDController(0.03, 0, 0, omegaConstraints); //to tune

  public AlignToReef() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommand = !RobotContainer.m_indexSubsystem.hasCoral();
    //RobotContainer.m_autoAlignCam.clearCameraFIFOBuffer();
    RobotContainer.m_autoAlignCam.resetTargetDetector();
    robotPose = RobotContainer.drivebase.getPose();

    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    omegaController.reset(robotPose.getRotation().getRadians());

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);

    if (RobotContainer.m_driverController.povLeft().getAsBoolean()) {
      reefOffset = VisionConstants.reefToAprilTagOffset; // to measure
    }
    else if (RobotContainer.m_driverController.povRight().getAsBoolean()) {
      reefOffset = -VisionConstants.reefToAprilTagOffset; // to measure
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    robotPose = RobotContainer.drivebase.getPose();
    final boolean cameraSawTarget = RobotContainer.m_autoAlignCam.cameraSawTarget();
    if (!cameraSawTarget) {
      RobotContainer.m_autoAlignCam.getCameraResults();
      RobotContainer.drivebase.driveFieldOriented(getDriveAngularVelocity()); 
    }
    else if (cameraSawTarget) {
      //System.out.println("HAWK TUAH!");
      closestAprilTagPose = RobotContainer.m_autoAlignCam.poseOfAprilTag;
      xController.setGoal(closestAprilTagPose.getX());
      yController.setGoal(closestAprilTagPose.getY() + reefOffset);
      omegaController.setGoal(closestAprilTagPose.getRotation().getRadians());
      
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        System.out.println("X Controller at goal");
        xSpeed = 0;
      }

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

      SmartDashboard.putNumber("Accumulated Y Error", yController.getAccumulatedError());
      SmartDashboard.putNumber("Accumulated X Error", xController.getAccumulatedError());

      
      RobotContainer.drivebase.drive(
        //ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
        ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.m_driverController.getLeftY(), ySpeed, omegaSpeed, robotPose.getRotation()));

    }
    
}
    
/*  
  public void executer() {
    System.out.println("HAWK TUAH!");
    final DoubleSupplier xTrans = () -> RobotContainer.m_autoAlignCam.getRobotToTagPose().getX();
    final DoubleSupplier yTrans = () -> (RobotContainer.m_autoAlignCam.getRobotToTagPose().getY());
    final DoubleSupplier maxAngularRotation = () -> RobotContainer.drivebase.getMaximumChassisAngularVelocity();
    //Commands.defer(() -> RobotContainer.drivebase.createPathToAprilTag(RobotContainer.m_autoAlignCam.getRobotToTagPose()), Set.of(RobotContainer.drivebase));
    //RobotContainer.drivebase.drive(new Translation2d(RobotContainer.m_autoAlignCam.getRobotToTagPose().getX(), RobotContainer.m_autoAlignCam.getRobotToTagPose().getY()), RobotContainer.m_autoAlignCam.getYaw(), true);
    //RobotContainer.m_autoAlignCam.setTagWayPoint();
    endCommand = true;   
  }
*/


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

  private SwerveInputStream getDriveAngularVelocity() {
    return SwerveInputStream.of(RobotContainer.drivebase.getSwerveDrive(), 
    () -> RobotContainer.m_driverController.getLeftY() * -1,
    () -> RobotContainer.m_driverController.getLeftX() * -1)
    .withControllerRotationAxis(() -> RobotContainer.m_driverController.getRightX() * -1)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);        
  }

}


