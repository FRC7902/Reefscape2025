// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rotateAlgaeArm extends Command {
  /** Creates a new rotateAlgaeArm. */
  public rotateAlgaeArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  //the button bindings for the algae arm are here 
    if (RobotContainer.m_operatorStick.getAButton()){
       RobotContainer.m_intake.pidArmDown();
    }

    if (RobotContainer.m_operatorStick.getYButton()){
       RobotContainer.m_intake.pidArmHorizontal();
    }

    if (RobotContainer.m_operatorStick.getXButton()){
     RobotContainer.m_intake.pidArmUp();
    }

    if (RobotContainer.m_operatorStick.getBButton()){
      RobotContainer.m_intake.armUp();
    }

    if (RobotContainer.m_operatorStick.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5){ //here, i wanna do something like 
                                                                                  //if the driver moves one of the joysticks up
                                                                                  //the arm moves up.
      RobotContainer.m_intake.armDown();
      
    }

    if (RobotContainer.m_operatorStick.getTriggerAxis(GenericHID.Hand.kRight) > 0.5){
      RobotContainer.m_intake.armStop(); 
      
    }

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
}
