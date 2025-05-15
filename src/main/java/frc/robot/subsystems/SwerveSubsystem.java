// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlanner;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.visions.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem(File directory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED,
                    new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
                            Rotation2d.fromDegrees(0)));
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        if (Robot.isSimulation()) {
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
        } else {

            swerveDrive.setCosineCompensator(true);// !SwerveDriveTelemetry.isSimulation); //
                                                   // Disables
                                                   // cosine compensation for
                                                   // simulations since it causes discrepancies not
                                                   // seen in real life.
            swerveDrive.setHeadingCorrection(false); // Heading correction should only be
            // used while
            // controlling the robot via
            // angle.

        }

        swerveDrive.setAngularVelocityCompensation(true, false, 0.08); // Correct for skew that
                                                                       // gets
                                                                       // worse as angular
                                                                       // velocity
                                                                       // increases. Start with a
                                                                       // coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to
                                                               // resynchronize your absolute
                                                               // encoders and motor encoders
                                                               // periodically when they are not
                                                               // moving.

        swerveDrive.setChassisDiscretization(false, true, 0.03);
        swerveDrive.swerveController.addSlewRateLimiters(null, null, null);
        swerveDrive.swerveController.setMaximumChassisAngularVelocity(20);

        setupPathPlanner();

        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyro));
    }

    /**
     * Setup the photon vision class.
     */
    // public void setupPhotonVision() {
    // vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Gyro angle rotation (rad)",
                swerveDrive.getGyro().getRotation3d().getAngle());

        SmartDashboard.putString("Robo Pose2D", swerveDrive.getPose().toString());

        // When vision is enabled we must manually update odometry in SwerveDrive
        // if (visionDriveTest) {

        // }
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(this::getPose,
                    // Robot pose supplier
                    this::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(speedsRobotRelative,
                                    swerveDrive.kinematics
                                            .toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for
                            // holonomic drive trains
                            new PIDConstants(PathPlanner.kPDrive, PathPlanner.kIDrive,
                                    PathPlanner.kDDrive),
                            // Translation PID constants
                            new PIDConstants(PathPlanner.kPAngle, PathPlanner.kIAngle,
                                    PathPlanner.kDAngle)
                    // Rotation PID constants
                    ), config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    }, this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }


    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(pathName);
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0,
                3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
    }

    public double getMaximumChassisAngularVelocity() {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(
                    SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                            0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3)
                            * swerveDrive.getMaximumChassisAngularVelocity(),
                    true, false);
        });
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset
     * when calling this method. However, if either gyro angle or module position is
     * reset, this
     * must be called in order for odometry to keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the
     * underlying drivebase. Note, this is not the raw gyro reading, this may be
     * corrected from
     * calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void alignRobotToAprilTag(double radiansHeading, double xTranslation, double yTranslation) {
        SwerveController controller = swerveDrive.getSwerveController();

        /*
         * swerveDrive.drive(ChassisSpeeds.fromRobotRelativeSpeeds(xTranslation,
         * yTranslation,
         * -controller.headingCalculate(swerveDrive.getOdometryHeading().unaryMinus().
         * getRadians(), new Rotation2d(radiansHeading).getRadians()),
         * swerveDrive.getPose().getRotation()));
         * 
         */
        swerveDrive.drive(new Translation2d(xTranslation, yTranslation),
                controller.headingCalculate(swerveDrive.getOdometryHeading().getRadians(), radiansHeading),
                false, false);

        SmartDashboard.putNumber("Odom Heading (rad)", swerveDrive.getOdometryHeading().unaryMinus().getRadians());
        SmartDashboard.putNumber("Target Heading (rad)", radiansHeading);
        SmartDashboard.putNumber("Error (rad)",
                new Rotation2d(radiansHeading).minus(swerveDrive.getOdometryHeading().unaryMinus()).getRadians());
    }

    public void strafe(double strafePower, double speedMultiplier) {
        swerveDrive.drive(new Translation2d(0,
                strafePower * Math.abs(speedMultiplier) * swerveDrive.getMaximumChassisVelocity()),
                0, false, false);

    }
}