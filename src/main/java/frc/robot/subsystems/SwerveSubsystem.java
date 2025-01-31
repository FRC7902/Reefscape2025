// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PathPlanner;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    private RobotConfig robotConfig;

    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem(File directory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                    new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
                            Rotation2d.fromDegrees(0)));
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while
        // controlling the robot via
        // angle.
        swerveDrive.setCosineCompensator(true);// !SwerveDriveTelemetry.isSimulation); // Disables
                                               // cosine compensation for
                                               // simulations since it causes discrepancies not
                                               // seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, -0.14); // Correct for skew that gets
                                                                       // worse as angular velocity
                                                                       // increases. Start with a
                                                                       // coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to
                                                               // resynchronize your absolute
                                                               // encoders and motor encoders
                                                               // periodically when they are not
                                                               // moving.

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a
                                     // starting
                                     // pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setChassisSpeed(speeds), // Method that will drive the
                                                                   // robot given
                                                                   // ROBOT RELATIVE ChassisSpeeds.
                                                                   // Also
                                                                   // optionally outputs individual
                                                                   // module
                                                                   // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                                                // following
                                                // controller for holonomic drive trains
                        new PIDConstants(PathPlanner.kPDrive, PathPlanner.kIDrive,
                                PathPlanner.kDDrive), // Translation
                                                      // PID
                                                      // constants
                        new PIDConstants(PathPlanner.kPAngle, PathPlanner.kIAngle,
                                PathPlanner.kDAngle) // Rotation
                                                     // PID
                                                     // constants
                ), robotConfig, // The robot configuration
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
                }, this // Reference to this subsystem to set requirements
        );


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Gyro angle rotation (rad)",
                swerveDrive.getGyro().getRotation3d().getAngle());
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
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

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX Heading X to calculate angle of the joystick.
     * @param headingY Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier headingX, DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                    scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a
     * rotation rate, and calculates and commands module states accordingly. Can use either
     * open-loop or closed-loop velocity control for the wheel velocities. Also has field- and
     * robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation {@link Translation2d} that is the commanded linear velocity of the robot,
     *        in meters per second. In robot-relative mode, positive x is torwards the bow (front)
     *        and positive y is torwards port (left). In field-relative mode, positive x is away
     *        from the alliance wall (field North) and positive y is torwards the left wall when
     *        looking through the driver station glass (field West).
     * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
     *        field/robot relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled
                                                                        // since it shouldn't be
                                                                        // used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
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
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
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
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeed) {
        swerveDrive.setChassisSpeeds(chassisSpeed);
    }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

