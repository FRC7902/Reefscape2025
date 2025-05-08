// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static class DemoConstants {
        public static final double DRIVE_SPEED_MULTIPLIER = 0.1;
        public static final double ALGAE_SHOOT_POWER_MULTIPLIER = 0.5;
        public static final double CORAL_SHOOT_POWER_MULTIPLIER = 0.5;
        public static final double ELEVATOR_SPEED_MULTIPLIER = 0.25;

    }

    public static class DriveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(15.0) * DemoConstants.DRIVE_SPEED_MULTIPLIER;
        public static final double kSlowDriveSpeedMultiplier = 0.5;
        public static final double kAutoAlignForwardBackwardSpeedMultiplier = 1.5;
    }

    public static class VisionConstants {
        // Camera Offset

        // y distance = 299.8 mm (right left)
        // x dist = 269.87 mm (forward back)
        // ground = 272.94 mm

        public static final int[] acceptedTagIDs =
                new int[] {2, 3, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

        public static final double kGroundToCamera = 0.27294; // meters
        public static final double kFowardToCamera = 0.26987; // meters
        public static final double kSidewaysToCamera = 0.2998; // meters
        public static final double kCameraRotation = 25.3; // degrees
        public static final String kCameraName = "limelight";

        public static double kLocalizationDisLim = 2;

        public static double kStdDevs = 0.800000;

        // //April Tag Offset
        // public static final double kGroundToAprilTagDistance = 0.171; //in meters
        // public static final double kAprilTagAreaLimit = 4.5;
        public static final double kAprilTagOffset = 0;

        // Reef Offset
        public static double leftReefToAprilTagOffset = -0.165000;
        public static double rightReefToAprilTagOffset = 0.210000;

        // PID Y Controller Constants
        public static final TrapezoidProfile.Constraints yConstraints =
                new TrapezoidProfile.Constraints(5*DemoConstants.DRIVE_SPEED_MULTIPLIER, 3);
        public static double yControllerTolerance = 0;
        public static double kPY = 0.07;
        public static double kIY = 0;
        public static double kDY = 0;

        public static double kPY2 = 6.0;
        public static double kIY2 = 0;
        public static double kDY2 = 0.5;

        public static int kSecondPIDControllerStartingPoint = 13; // to change

    }

    public static class PathPlanner {
        public static final double kPDrive = 1.95;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0.01;

        public static final double kPAngle = 2.6;
        public static final double kIAngle = 0;
        public static final double kDAngle = 0.01;

    }

    public static class ClimbConstants {
        // CAN ID of the primary motor
        public static final int kClimbLeaderMotorCANID = 41;
        // CAN ID of the secondary motor
        public static final int kClimbFollowerMotorCANID = 42;
        // Digital IO Port that the REV Through Bore Absolute Encoder is connected to on
        // the RIO (uses PWM)
        public static final int kRevThroughBoreIO = 0;
        // current limit when motor rpm is at 0 (in amps)
        public static final int kMotorStallCurrent = 50;
        // current limit when motor rpm is at 5700 (unique to NEO motors) (in amps)
        public static final int kMotorFreeSpeedCurrent = 50;
        // climb arm angle limiter when arm is moving outwards from the robot (in
        // degrees)
        public static final double kClimbBackwardLimit = 0;
        // climb arm angle limiter when arm is moving inwards to the robot (in degrees)
        public static final double kClimbForwardLimit = 263.1;
        public static final double kClimbHomePose = 315.1;
        // motor voltage applied when climb arm is set to move foward
        public static final double kMotorVoltageUp = 12;
        // motor voltage applied when climb arm is set to move backwards (negative to
        // move the opposite direction)
        public static final double kMotorVoltageDown = -12;
        // 975.6 Hz for REV Through Bore Absolute Encoder
        // https://www.revrobotics.com/rev-11-1271/#:~:text=Output%20Frequency%3A%20975.6Hz
        public static final double kRevThroughBoreFrequency = 975.6;
        // The REV Throughbore has a minimum pulse of 1 μs / 1025 μs and a maximum pulse
        // of 1024 μs / 1025 μs
        // these values are used to ensure accuracy of the encoder's data
        public static final double kRevThroughBoreMinPulse = 0.000975609756;
        public static final double kRevThroughBoreMaxPulse = 0.99902439;
        public static final int kLeftServoID = 9;
        public static final int kRightServoID = 8;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        // Joystick Deadband
        // Also make sure to set the angleJoystickRadiusDeadband in
        // controllerproperties.json
        public static final double DEADBAND = 0.15;
    }

    public static class AlgaeManipulatorConstants {
        public static final int kMotorCANId = 30;
        public static final int kMotorCurrentLimit = 60;

        public static final int kbeamBreakPortId = 30;

        public static final int kIntakeVoltage = 12;
        public static final double kOuttakeVoltage = -12.0 * DemoConstants.ALGAE_SHOOT_POWER_MULTIPLIER;

    }

    public static class CoralIndexerConstants {
        public static int kIndexMotorCAN = 35;
        public static final double kRampRate = 0;

        public static final double kOuttakePower = 0.75 * DemoConstants.CORAL_SHOOT_POWER_MULTIPLIER;
        public static final double kL1OuttakePower = 0.4;
        public static final double kIntakePower = 1;
        public static final double kCorrectionPower = 0.2;

        public static double kS = 1;
        public static double kV = 1;

        public static final int kShallowBeamBreakPort = 8;
        public static final int kDeepBeamBreakPort = 9;

    }

    public static final class ElevatorConstants {
        // CAN IDs
        public static final int kElevatorLeaderCAN = 54;
        public static final int kElevatorFollowerCAN = 55;

        // Physical Constants
        public static final double kElevatorGearing = 7.5;
        public static final double kElevatorCarriageMass = Units.lbsToKilograms(20);
        public static final double kElevatorDrumRadius = Units.inchesToMeters(1.644 / 2);
        public static final double kElevatorMetersPerMotorRotation =
                (kElevatorDrumRadius * 2 * Math.PI) / kElevatorGearing;

        // Elevator Dimensions
        public static final double kElevatorHeightMeters = 0.0;
        public static final double kElevatorMinHeightMeters = 0.0;
        public static final double kElevatorMaxHeightMeters = 0.90;

        // Motion Constraints
        public static final double kElevatorMaxVelocity =
                1.5 / ElevatorConstants.kElevatorMetersPerMotorRotation
                        * DemoConstants.ELEVATOR_SPEED_MULTIPLIER;
        public static final double kElevatorMaxAcceleration = 160.0;

        // PID Constants
        public static final double kElevatorP = 1;
        public static final double kElevatorI = 0.0;
        public static final double kElevatorD = 0.01;

        // Elevator Gains
        // set all to 0 during testing
        public static final double kElevatorS = 0.0; // negligible
        public static final double kElevatorG = 0.2;
        public static final double kElevatorV = 6.85 * kElevatorMetersPerMotorRotation;
        public static final double kElevatorA = 0.04 * kElevatorMetersPerMotorRotation;

        // ===== Elevator Setpoints =====
        public static final double kElevatorCoralStationAndProcessorHeight = 0.0;

        public static final double kElevatorCoralLevel1StartHeight = 0.025;
        public static final double kElevatorCoralLevel1EndHeight = 0.225;
        public static final double kElevatorCoralLevel2Height = 0.188;
        public static final double kElevatorCoralLevel3Height = 0.548;

        public static final double kElevatorAlgaeLowHeight = 0.604;
        public static final double kElevatorAlgaeHighHeight = 0.90;
        // ==============================

        // ===== Control Parameters =====
        public static final double kElevatorTargetError = 0.005;
        public static final double kElevatorMotorResistance = 0.002; // Assume 2mOhm resistance for
                                                                     // voltage drop calculation
    }
}
