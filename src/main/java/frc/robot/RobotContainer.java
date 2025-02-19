// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.teleop.algae_manipulator.IntakeAlgaeCommand;
import frc.robot.commands.teleop.algae_manipulator.OuttakeAlgaeCommand;
import frc.robot.commands.teleop.climb.MoveClimbDownCommand;
import frc.robot.commands.teleop.climb.MoveClimbUpCommand;
import frc.robot.commands.teleop.coral_indexer.CorrectCoralPositionCommand;
import frc.robot.commands.teleop.coral_indexer.IntakeCoralCommand;
import frc.robot.commands.teleop.coral_indexer.OuttakeCoralCommand;
import frc.robot.commands.teleop.elevator.RelativeMoveElevatorCommand;
import frc.robot.commands.teleop.elevator.SetElevatorPositionCommand;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final AlgaeManipulatorSubsystem m_algaeElevatorManipulatorSubsystem =
            new AlgaeManipulatorSubsystem();
    // The robot's subsystems and commands are defined here...
    public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public static final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

    public static final CoralIndexerSubsystem m_indexSubsystem = new CoralIndexerSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    private final SwerveSubsystem drivebase =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
     * velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream
            .of(drivebase.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
            .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */
    SwerveInputStream driveRobotOriented =
            driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
            .of(drivebase.getSwerveDrive(), () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX())
            .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
            .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
            .allianceRelativeControl(true);

    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(
                    () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        // Swerve drive controls
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity =
                drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity =
                drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedDirectAngleKeyboard =
                drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard =
                drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

        // Default to field-centric swerve drive
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // Zero gyro
        m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));


        // Raise elevator (by height of Algae diameter) while intaking algae
        m_driverController.leftBumper().onTrue(
                Commands.race(new IntakeAlgaeCommand(), new RelativeMoveElevatorCommand(0.41)));
        m_driverController.rightBumper().whileTrue(new OuttakeAlgaeCommand());

        m_indexSubsystem.setDefaultCommand(
                new IntakeCoralCommand(Constants.CoralIndexerConstants.kIntakePower)
                        .andThen(new CorrectCoralPositionCommand()).andThen(new IntakeCoralCommand(
                                Constants.CoralIndexerConstants.kCorrectionPower)));

        // Coral indexer controls (default intake and manual outtake)
        m_driverController.rightTrigger().whileTrue(new OuttakeCoralCommand());

        // Elevator coral positions
        m_operatorController.x().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralStationHeight));
        m_operatorController.a().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel1Height));
        m_operatorController.b().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height));
        m_operatorController.y().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height));

        // Elevator algae positions
        m_operatorController.povDown()
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight));
        m_operatorController.povUp()
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight));

        // Climb controls
        m_driverController.leftStick().whileTrue(new MoveClimbUpCommand());
        m_driverController.rightStick().whileTrue(new MoveClimbDownCommand());

        // Elevator adjustment for tuning
        m_driverController.povUp().whileTrue(new RelativeMoveElevatorCommand(0.00635));
        m_driverController.povDown().whileTrue(new RelativeMoveElevatorCommand(-0.00635));
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
