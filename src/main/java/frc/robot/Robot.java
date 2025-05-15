
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CoralIndexerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.teleop.SwerveCommands.StrafeLeftCommand;
import frc.robot.commands.teleop.SwerveCommands.StrafeRightCommand;
import frc.robot.commands.teleop.algae_manipulator.IntakeAlgaeCommand;
import frc.robot.commands.teleop.algae_manipulator.OuttakeAlgaeCommand;
import frc.robot.commands.teleop.climb.InitiateClimbCommand;
import frc.robot.commands.teleop.climb.LockFunnelCommand;
import frc.robot.commands.teleop.climb.ManualClimb;
import frc.robot.commands.teleop.climb.MoveClimbAttackAngleCommand;
import frc.robot.commands.teleop.climb.MoveClimbUpCommand;
import frc.robot.commands.teleop.coral_indexer.AutomaticIntakeCoralCommand;
import frc.robot.commands.teleop.coral_indexer.ManualIntakeCoralCommand;
import frc.robot.commands.teleop.coral_indexer.OuttakeCoralCommand;
import frc.robot.commands.teleop.coral_indexer.WaitForCoral;
import frc.robot.commands.teleop.elevator.SetElevatorPositionCommand;
import frc.robot.commands.teleop.visions.AlignToReef;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.visions.CameraInterface;
import frc.robot.visions.ReefSide;
import swervelib.SwerveInputStream;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public static final AlgaeManipulatorSubsystem m_algaeElevatorManipulatorSubsystem =
            new AlgaeManipulatorSubsystem();
    // The robot's subsystems and commands are defined here...
    public static final CoralIndexerSubsystem m_indexSubsystem = new CoralIndexerSubsystem();
    public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public static final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();


    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public static final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public static final SwerveSubsystem m_swerveSubsystem =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    private final SendableChooser<Command> autoChooser;

    public static final CameraInterface m_cameraSubsystem =
            new CameraInterface(VisionConstants.kCameraName, m_swerveSubsystem);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        configureBindings();
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("DEFAULT");
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Swerve drive controls


        m_elevatorSubsystem.createElevatorCommands();
        m_swerveSubsystem.createSwerveDriveCommands();

        // Default to field-centric swerve drive
        // m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveFieldOrientedAnglularVelocity);

        m_climbSubsystem.setDefaultCommand(new LockFunnelCommand());

        // Zero gyro
        m_driverController.start().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));

        m_driverController.back()
                .whileTrue(Commands.parallel(new OuttakeAlgaeCommand(), new OuttakeCoralCommand()));

        // m_operatorController.back().whileTrue(new InitiateClimbCommand());
        m_operatorController.start()
                .onTrue(new SequentialCommandGroup(new SetElevatorPositionCommand(
                        ElevatorConstants.kElevatorCoralStationAndProcessorHeight).withTimeout(3),
                        new InstantCommand(() -> {
                        m_swerveSubsystem.driveAngularVelocity
                                    .scaleTranslation(DriveConstants.kSlowDriveSpeedMultiplier);
                        }), new InitiateClimbCommand().withTimeout(1),
                        new MoveClimbAttackAngleCommand(m_climbSubsystem)));
        m_operatorController.back().whileTrue(m_swerveSubsystem.centerModulesCommand());

        // Raise elevator (by height of Algae diameter) while intaking algae
        // m_driverController.leftBumper().whileTrue(m_whileTrueSelectIntakeCommand);
        m_driverController.leftBumper()
                .onTrue(new ParallelCommandGroup(
                        m_elevatorSubsystem.m_whileTrueSelectIntakeCommand
                                .until(() -> !m_driverController.leftBumper().getAsBoolean()),
                        m_elevatorSubsystem.m_onTrueSelectIntakeCommand));
        m_driverController.rightBumper().whileTrue(m_elevatorSubsystem.m_selectOuttakeCommand);

        // Strafe controls
        // m_driverController.leftTrigger(0.05).whileTrue(new SequentialCommandGroup(new
        // CheckForAprilTag(0), new AlignToReef(this, 0)));
        // m_driverController.rightTrigger(0.05).whileTrue(new
        // SequentialCommandGroup(new CheckForAprilTag(1), new AlignToReef(this, 1)));

        m_driverController.a().whileTrue(new AlignToReef(m_cameraSubsystem, m_swerveSubsystem, m_driverController, ReefSide.LEFT)); // left
        m_driverController.b().whileTrue(new AlignToReef(m_cameraSubsystem, m_swerveSubsystem, m_driverController, ReefSide.LEFT)); //right

        // Climb controls
        m_driverController.povUp()
                .whileTrue(new ConditionalCommand(new ManualClimb(m_climbSubsystem, 12),
                        Commands.none(), m_climbSubsystem::isFunnelUnlocked));
        m_driverController.povDown()
                .whileTrue(new ConditionalCommand(new ManualClimb(m_climbSubsystem, -12),
                        Commands.none(), m_climbSubsystem::isFunnelUnlocked));

        m_driverController.povLeft()
                .onTrue(new ConditionalCommand(new MoveClimbUpCommand(m_climbSubsystem),
                        Commands.none(), m_climbSubsystem::isFunnelUnlocked));
        m_driverController.povRight()
                .onTrue(new ConditionalCommand(new MoveClimbAttackAngleCommand(m_climbSubsystem),
                        Commands.none(), m_climbSubsystem::isFunnelUnlocked));

        m_driverController.leftTrigger(0.05).whileTrue(new StrafeLeftCommand());
        m_driverController.rightTrigger(0.05).whileTrue(new StrafeRightCommand());

        m_indexSubsystem.setDefaultCommand(
                new AutomaticIntakeCoralCommand(CoralIndexerConstants.kIntakePower));

        // Elevator coral positions
        m_operatorController.x()
                .onTrue(new ConditionalCommand(
                        new SetElevatorPositionCommand(
                                ElevatorConstants.kElevatorCoralLevel1StartHeight),
                        Commands.none(), m_indexSubsystem::hasCoral));
        m_operatorController.a().onTrue(new SetElevatorPositionCommand(
                ElevatorConstants.kElevatorCoralStationAndProcessorHeight));
        m_operatorController.b()
                .onTrue(new ConditionalCommand(
                        new SetElevatorPositionCommand(
                                ElevatorConstants.kElevatorCoralLevel2Height),
                        Commands.none(), m_indexSubsystem::hasCoral));
        m_operatorController.y()
                .onTrue(new ConditionalCommand(
                        new SetElevatorPositionCommand(
                                ElevatorConstants.kElevatorCoralLevel3Height),
                        Commands.none(), m_indexSubsystem::hasCoral));

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

        NamedCommands.registerCommand("WaitForCoral", new WaitForCoral());


        /*
         * NamedCommands.registerCommand("ElevatorL2", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height));
         * NamedCommands.registerCommand("Elevator L3", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height));
         * NamedCommands.registerCommand("Intake Algae", new IntakeAlgaeCommand());
         * NamedCommands.registerCommand("Coral Intake", new
         * IntakeCoralCommand(Constants.CoralIndexerConstants.kIntakePower));
         * NamedCommands.registerCommand("Coral Correcter", new
         * CorrectCoralPositionCommand());
         * NamedCommands.registerCommand("Coral Outake", new OuttakeCoralCommand());
         * NamedCommands.registerCommand("Low Algae", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight));
         * NamedCommands.registerCommand("High Algae", new
         * SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight));
         * NamedCommands.registerCommand("Lowest Height", new
         * SetElevatorPositionCommand(0));
         */
        // NamedCommands.registerCommand("OutakeCoralV2",new
        // OuttakeCoralCommand(Constants.CoralIndexerConstants.kL1OuttakePower));
        // NamedCommands.registerCommand("StopCoralOutake", new OuttakeCoralCommand(0));

        // preloads the path

        // Register Event Triggers
        new EventTrigger("ZeroPosition").onTrue(new SetElevatorPositionCommand(
                ElevatorConstants.kElevatorCoralStationAndProcessorHeight));
        new EventTrigger("ElevatorL1").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel1StartHeight));
        new EventTrigger("ElevatorL2").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height));
        new EventTrigger("ElevatorL3").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height));
        new EventTrigger("lowalgae")
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight));
        new EventTrigger("highalgae")
                .onTrue(new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight));

        new EventTrigger("ElevatorL1WithWait").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel1StartHeight,
                        true));
        new EventTrigger("ElevatorL2WithWait").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel2Height, true));
        new EventTrigger("ElevatorL3WithWait").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorCoralLevel3Height, true));
        new EventTrigger("lowalgaeWithWait").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeLowHeight, true));
        new EventTrigger("highalgaeWithWait").onTrue(
                new SetElevatorPositionCommand(ElevatorConstants.kElevatorAlgaeHighHeight, true));

        new EventTrigger("intakealgaeon").onTrue(new IntakeAlgaeCommand().withTimeout(1.5));
        new EventTrigger("outtakealgaeon").onTrue(new OuttakeAlgaeCommand().withTimeout(3));

        // new EventTrigger("intakealgaeoff").toggleOnFalse(new IntakeAlgaeCommand());
        new EventTrigger("coraloutakeon")
                .onTrue(new OuttakeCoralCommand(1)
                        .until(() -> !m_indexSubsystem.isDeepBeamBroken()));
        // new EventTrigger("coraloutakeoff").toggleOnFalse(new OuttakeCoralCommand());

        // new EventTrigger("shoot note").and(new
        // Trigger(exampleSubsystem::someCondition)).onTrue(Commands.print("shoot
        // note");

        // Point Towards Zone Triggers
        // new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at
        // speaker"));
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_swerveSubsystem.getSwerveDrive().setMotorIdleMode(true);
        m_autonomousCommand = autoChooser.getSelected();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_swerveSubsystem.getSwerveDrive().setMotorIdleMode(true);

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
