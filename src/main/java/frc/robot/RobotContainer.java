// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetShootSpeed;
import frc.robot.commands.elevator.ElevatorReefSetpoint;
import frc.robot.commands.teleop.IntakeAlgaeCommand;
import frc.robot.commands.teleop.IntakeCoralCommand;
import frc.robot.commands.teleop.NullCommand;
import frc.robot.commands.teleop.OuttakeAlgaeCommand;
import frc.robot.commands.teleopCommands.climb.MoveClimbDown;
import frc.robot.commands.teleopCommands.climb.MoveClimbUp;
import frc.robot.subsystems.AlgaeElevatorManipulatorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

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
    public static final AlgaeElevatorManipulatorSubsystem m_algaeElevatorManipulatorSubsystem = new AlgaeElevatorManipulatorSubsystem();
    // The robot's subsystems and commands are defined here...
    public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

    public static final IndexSubsystem m_indexSubsystem = new IndexSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(m_driverController::getRightX,
                    m_driverController::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX())
            .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    m_driverController.getRawAxis(
                            2) *
                            Math.PI)
                    *
                    (Math.PI *
                            2),
                    () -> Math.cos(
                            m_driverController.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2))
            .headingWhile(true);

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
        // Swerve drive controls
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                .driveFieldOriented(driveAngularVelocityKeyboard);

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

        m_driverController.leftBumper().whileTrue(new IntakeAlgaeCommand());
        m_driverController.rightBumper().whileTrue(new OuttakeAlgaeCommand());

        m_indexSubsystem.setDefaultCommand(
                new ConditionalCommand(new IntakeCoralCommand(), new NullCommand(),
                        m_indexSubsystem::isBeamBroken));

        m_driverController.rightTrigger().whileTrue(new SetShootSpeed());

        m_operatorController.a().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kLevel1));
        m_operatorController.b().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kLevel2));
        m_operatorController.y().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kLevel3));
        m_operatorController.x().onTrue(new ElevatorReefSetpoint(ElevatorConstants.kElevatorMinHeightMeters));

        // runs the climb motors up when the up button is pressed on the POV buttons of
        // the controller.
        m_operatorController.povUp().whileTrue(new MoveClimbUp(m_ClimbSubsystem, this));
        // runs the climb motors down when the down button is pressed on the POV buttons
        // of the controller.
        m_operatorController.povDown().whileTrue(new MoveClimbDown(m_ClimbSubsystem, this));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}