// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class spinAlgaeArm extends Command {
  /** Creates a new spinAlgaeArm. */


  public spinAlgaeArm() {
    addRequirements(RobotContainer.m_intake);
    // Use addRequirements() here to declare subsystem dependencies.

  
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.m_operatorStick.getAButton()){
      RobotContainer.m_intake.algaeOuttake();
    }


    if (RobotContainer.m_operatorStick.getXButton()){
      RobotContainer.m_intake.algaeIntake();
    }

    
    if (RobotContainer.m_operatorStick.getYButton()){
      RobotContainer.m_intake.algaeStop();
    }
   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_intake.algaeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.m_intake.hasAlgae()) {
      return true;
    }

    else {
      return false;
  }
  }
}
