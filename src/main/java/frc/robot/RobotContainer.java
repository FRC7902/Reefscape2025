// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.teleop.NullCommand;
import frc.robot.commands.teleop.SwerveCommands.StrafeLeftCommand;
import frc.robot.commands.teleop.SwerveCommands.StrafeRightCommand;
import frc.robot.commands.teleop.algae_manipulator.IntakeAlgaeCommand;
import frc.robot.commands.teleop.algae_manipulator.OuttakeAlgaeCommand;
import frc.robot.commands.teleop.climb.InitiateClimbCommand;
import frc.robot.commands.teleop.climb.LockFunnelCommand;
import frc.robot.commands.teleop.climb.MoveClimbDownCommand;
import frc.robot.commands.teleop.climb.MoveClimbUpCommand;
import frc.robot.commands.teleop.coral_indexer.CorrectCoralPositionCommand;
import frc.robot.commands.teleop.coral_indexer.IntakeCoralCommand;
import frc.robot.commands.teleop.coral_indexer.OuttakeCoralCommand;
import frc.robot.commands.teleop.elevator.SetElevatorPositionCommand;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.visions.CameraInterface;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.teleop.climb.ManualClimb;
import frc.robot.commands.teleop.visions.AlignToReef;
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
    public static final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public static final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public static final SwerveSubsystem m_swerveSubsystem =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    public static final CameraInterface m_autoAlignCam = new CameraInterface("skibidi", drivebase);         

    private final SendableChooser<Command> autoChooser;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
     * velocity.
     */
    public SwerveInputStream driveAngularVelocity = SwerveInputStream
            .of(drivebase.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
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
            .of(m_swerveSubsystem.getSwerveDrive(), () -> -m_driverController.getLeftY(),
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

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("DEFAULT");

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        /*
         * NamedCommands.registerCommand("ElevatorL2", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height));
         * NamedCommands.registerCommand("Elevator L3", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height));
         * NamedCommands.registerCommand("Intake Algae", new IntakeAlgaeCommand());
         * NamedCommands.registerCommand("Coral Intake", new
         * IntakeCoralCommand(Constants.CoralIndexerConstants.kIntakePower));
         * NamedCommands.registerCommand("Coral Correcter", new CorrectCoralPositionCommand());
         * NamedCommands.registerCommand("Coral Outake", new OuttakeCoralCommand());
         * NamedCommands.registerCommand("Low Algae", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight));
         * NamedCommands.registerCommand("High Algae", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight));
         * NamedCommands.registerCommand("Lowest Height", new SetElevatorPositionCommand(0));
         */
        NamedCommands.registerCommand("OutakeCoralV2", new OuttakeCoralCommand(Constants.CoralIndexerConstants.kL1OuttakePower));
        NamedCommands.registerCommand("StopCoralOutake", new OuttakeCoralCommand(0));
        
        // preloads the path

        // Register Event Triggers
        new EventTrigger("ZeroPosition")
        .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralStationAndProcessorHeight));
        new EventTrigger("ElevatorL1").onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel1Height));
        new EventTrigger("ElevatorL2").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height));
        new EventTrigger("ElevatorL3").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height));
        new EventTrigger("intakealgaeon").onTrue(new IntakeAlgaeCommand().withTimeout(1.5));
        // new EventTrigger("intakealgaeoff").toggleOnFalse(new IntakeAlgaeCommand());
        new EventTrigger("coraloutakeon").onTrue(
                new OuttakeCoralCommand(Constants.CoralIndexerConstants.kL1OuttakePower).withTimeout(3));
        //new EventTrigger("coraloutakeoff").toggleOnFalse(new OuttakeCoralCommand());
        new EventTrigger("lowalgae")
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight));
        new EventTrigger("highalgae")
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight));



        // new EventTrigger("shoot note").and(new
        // Trigger(exampleSubsystem::someCondition)).onTrue(Commands.print("shoot note");

        // Point Towards Zone Triggers
        // new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at
        // speaker"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private ElevatorPosition select() {
        return m_elevatorSubsystem.getElevatorEnumPosition();
    }

    private final Command m_selectIntakeCommand = new SelectCommand<>(
            Map.ofEntries(Map.entry(ElevatorPosition.ALGAE_LOW, new IntakeAlgaeCommand()),
                    Map.entry(ElevatorPosition.ALGAE_HIGH, new IntakeAlgaeCommand())),

            this::select);

    private final Command m_selectOuttakeCommand = new SelectCommand<>(Map.ofEntries(
            Map.entry(ElevatorPosition.CORAL_L1, new OuttakeCoralCommand(Constants.CoralIndexerConstants.kL1OuttakePower)),
            Map.entry(ElevatorPosition.CORAL_L2, new OuttakeCoralCommand()),
            Map.entry(ElevatorPosition.CORAL_L3, new OuttakeCoralCommand()),
            Map.entry(ElevatorPosition.CORAL_STATION_AND_PROCESSOR, new OuttakeAlgaeCommand())),
            this::select);

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
        Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity =
                m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity =
                m_swerveSubsystem.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedDirectAngleKeyboard =
                m_swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard =
                m_swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);

        // Default to field-centric swerve drive
        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_climbSubsystem.setDefaultCommand(new LockFunnelCommand());

        // Zero gyro
        m_driverController.start().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));

        m_driverController.back()
                .whileTrue(Commands.parallel(new OuttakeAlgaeCommand(), new OuttakeCoralCommand()));

        m_operatorController.back().whileTrue(new InitiateClimbCommand());
        m_operatorController.start().whileTrue(m_swerveSubsystem.centerModulesCommand());

        // Raise elevator (by height of Algae diameter) while intaking algae
        m_driverController.leftBumper().whileTrue(m_selectIntakeCommand);
        m_driverController.rightBumper().whileTrue(m_selectOuttakeCommand);

        // Strafe controls
        m_driverController.leftTrigger(0.05).whileTrue(new StrafeLeftCommand());
        m_driverController.rightTrigger(0.05).whileTrue(new StrafeRightCommand());

        // Climb controls
        m_driverController.povUp()
                .whileTrue(new ConditionalCommand(new ManualClimb(m_climbSubsystem, 12),
                        new NullCommand(), m_climbSubsystem::isFunnelUnlocked));
        m_driverController.povDown()
                .whileTrue(new ConditionalCommand(new ManualClimb(m_climbSubsystem, -12),
                        new NullCommand(), m_climbSubsystem::isFunnelUnlocked));
        /* 
        m_driverController.povLeft()
                .onTrue(new ConditionalCommand(new MoveClimbUpCommand(m_climbSubsystem),
                        new NullCommand(), m_climbSubsystem::isFunnelUnlocked));
        m_driverController.povRight()
                .onTrue(new ConditionalCommand(new MoveClimbDownCommand(m_climbSubsystem),
                        new NullCommand(), m_climbSubsystem::isFunnelUnlocked));
        */
        m_driverController.povRight().whileTrue(new AlignToReef(drivebase, m_autoAlignCam, m_driverController, this, m_indexSubsystem));
        
        m_indexSubsystem
                .setDefaultCommand(
                        new IntakeCoralCommand(Constants.CoralIndexerConstants.kIntakePower)
                                .andThen(new CorrectCoralPositionCommand().withTimeout(1))
                                .andThen(new IntakeCoralCommand(
                                        Constants.CoralIndexerConstants.kCorrectionPower)
                                                .withTimeout(1)));

        // Elevator coral positions
        m_operatorController.x().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel1Height));
        m_operatorController.a().onTrue(new SetElevatorPositionCommand(
                ElevatorConstants.kElevatorCoralStationAndProcessorHeight));
        m_operatorController.b().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height));
        m_operatorController.y().onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height));

        // Elevator algae positions
        m_operatorController.povDown()
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight));
        m_operatorController.povUp()
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight));

        // ======= Test bindings =======

        // Micro elevator adjustment
        // m_driverController.povUp().whileTrue(new
        // RelativeMoveElevatorCommand(0.00635));
        // m_driverController.povDown().whileTrue(new
        // RelativeMoveElevatorCommand(-0.00635));

        // Snap to angle
        // m_driverController.a().onTrue(drivebase.snapToAngle(90, 1));
        // m_driverController.b().onTrue(drivebase.snapToAngle(180, 1));
        // m_driverController.y().onTrue(drivebase.snapToAngle(0, 1));
        // m_driverController.x().onTrue(drivebase.snapToAngle(270, 1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return drivebase.getAutonomousCommand("New Auto");

        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        m_swerveSubsystem.setMotorBrake(brake);
    }
}
