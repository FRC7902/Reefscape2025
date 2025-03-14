// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.visions;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.visions.CameraInterface;

public class AlignToReef extends Command {
  /** Creates a new AlignToReefCommand. */
  private boolean endCommand = false;

  private ProfiledPIDController yController;
  private ProfiledPIDController y2Controller;

  private CameraInterface m_autoAlignCam;
  private double aprilTagRotation;

  private double aprilTagID;

  private double aprilTagDistance;

  private final RobotContainer m_RobotContainer;

  private final int triggerPressed;
  

  public AlignToReef(CameraInterface m_autoAlignCamera, RobotContainer m_RobotContainer, int triggerPressed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerveSubsystem);
    this.m_autoAlignCam = m_autoAlignCamera;
    this.m_RobotContainer = m_RobotContainer;
    this.triggerPressed = triggerPressed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //endCommand = !RobotContainer.m_indexSubsystem.hasCoral();

    aprilTagRotation = m_autoAlignCam.getAprilTagRotation();
    aprilTagID = m_autoAlignCam.getTargetAprilTagID();

    y2Controller = new ProfiledPIDController(VisionConstants.kPY2, VisionConstants.kIY2, VisionConstants.kDY2, VisionConstants.yConstraints); //to tune

    if (Math.abs(aprilTagDistance) < VisionConstants.kSecondPIDControllerStartingPoint) {
      yController = y2Controller;
    }

    else {
      yController = new ProfiledPIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY, VisionConstants.yConstraints); //to tune
    }

    yController.reset(aprilTagDistance);
    yController.setTolerance(VisionConstants.yControllerTolerance);
    yController.setGoal(VisionConstants.kAprilTagOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override 
  public void execute() {
    aprilTagDistance = m_autoAlignCam.getTx();
    var ySpeed = yController.calculate(aprilTagDistance);
    if (yController.atGoal()) {
      System.out.println("Y Controller at Goal");
      ySpeed = 0;
    }
    
    hawkTuah("Accumulated Y Error", yController.getAccumulatedError());

    RobotContainer.m_swerveSubsystem.alignRobotToAprilTag(aprilTagRotation, getDriverControllerLeftY(), ySpeed, 0.5);
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
    return -RobotContainer.m_driverController.getLeftY();
  }

  public void hawkTuah(String text, double key) {
    SmartDashboard.putNumber(text, key);
  }
}