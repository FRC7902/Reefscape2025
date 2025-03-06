// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;

import org.dyn4j.geometry.Transform;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class GoToAprilTag extends Command {

    private final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
    private final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
    private final TrapezoidProfile.Constraints omegaConstraints =   new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints); //to tune
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints); //to tune
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, omegaConstraints); //to tune

    private Pose2d closestAprilTagPose;
    private Pose2d robotPose;
    private int closestAprilTagID;
    private boolean endCommand = false;

    private CameraInterface m_autoAlignCam;
    private SwerveSubsystem drivebase;
    private CoralIndexerSubsystem m_indexSubsystem;

    private double targetYaw = 0;
    private Pose2d poseOfAprilTag;

    private PhotonTrackedTarget lastTarget;


    private static final Transform3d TAG_TO_GOAL = 
    new Transform3d(
        new Translation3d(1.5, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI));

    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();


public GoToAprilTag(CameraInterface m_autoAlignCam, SwerveSubsystem drivebase, CoralIndexerSubsystem m_indexSubsystem) {
    addRequirements(RobotContainer.drivebase);
    this.m_autoAlignCam = m_autoAlignCam;
    this.drivebase = drivebase;
    this.m_indexSubsystem = m_indexSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = drivebase.getPose();
    endCommand = !m_indexSubsystem.hasCoral();
    m_autoAlignCam.resetTargetDetector();
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    omegaController.reset(robotPose.getRotation().getRadians());
    lastTarget = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    var robotPose2d = drivebase.getPose();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = m_autoAlignCam.camera.getLatestResult();
    int aprilTagID = -1;
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> m_autoAlignCam.isReefAprilTag(t.getFiducialId()))
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      
      aprilTagID = targetOpt.get().getFiducialId();

      Pose3d aprilTagPose = m_autoAlignCam.aprilTagFieldLayout.getTagPose(aprilTagID).get();

      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal

        Transform3d tagTransform = new Transform3d(aprilTagPose.getX(), aprilTagPose.getY(), aprilTagPose.getZ(), null);

        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget != null) {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      drivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
    
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
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
