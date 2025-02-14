// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevator.ElevatorReefSetpoint;
import frc.robot.subsystems.ElevatorSubsystem;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  private final Joystick m_joystick = new Joystick(0);

  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // test setpoints one at a time
    // new JoystickButton(m_joystick, 1)
    //     .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setPosition(
    //         ElevatorConstants.kLevel1)));

    // new JoystickButton(m_joystick, 2)
    // .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setPosition(
    // ElevatorConstants.kLevel2)));

    // new JoystickButton(m_joystick, 3)
    // .onTrue(new InstantCommand(() -> m_elevatorSubsystem.setPosition(
    // ElevatorConstants.kLevel3)));

    // manually reset the elevator
    // new JoystickButton(m_joystick, 4)
    //     .onTrue(new InstantCommand(() -> m_elevatorSubsystem.zero()));

    m_operatorController.a().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kLevel1));
    m_operatorController.b().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kLevel2));
    m_operatorController.y().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kLevel3));

    // SYSID COMMANDS
    // Run the SignalLogger when the left bumper is pressed, stop when the right bumper is pressed
    m_operatorController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    m_operatorController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dyanmic reverse
     */
    m_operatorController.y().whileTrue(m_elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_operatorController.a().whileTrue(m_elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_operatorController.b().whileTrue(m_elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_operatorController.x().whileTrue(m_elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
