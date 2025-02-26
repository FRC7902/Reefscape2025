// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.teleop.elevator.SetElevatorPositionCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitiateClimbCommand extends SequentialCommandGroup {
  /** Creates a new InitiateClimbCommand. */
  public InitiateClimbCommand() {
    
    addCommands(
      new SetElevatorPositionCommand(ElevatorConstants.kElevatorMinHeightMeters).withTimeout(1),
      new ParallelCommandGroup(
        new UnlockFunnelCommand(),  
        new ReadyClimberAngleCommand() 
      )
    );
  }
}
