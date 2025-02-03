// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound // TODO: Change this to
                                                                   // actual robot mass
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag // TODO: Check if this is
                                               // correct
  public static final double MAX_SPEED = Units.feetToMeters(10);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Joystick Deadband
    public static final double DEADBAND = 0.5;
  }

  public static class AlgaeElevatorManipulatorConstants {
    public static final int kMotorCANId = 30;
    public static final int kMotorCurrentLimit = 60;

    public static final int kbeamBreakPortId = 8;

    public static final int kIntakeVoltage = 12;
    public static final int kOuttakeVoltage = -12;
  }

  public static class IndexConstants {
    public static int kIndexMotorCAN = 38;
    public static final double kRampRate = 0;
    public static final double kShootSpeed = 40;
    public static double kS = 1;
    public static double kV = 1;
    public static final int kBeamSensorPort = 9;

  }

  public static class ElevatorConstants {
    public static final int kElevatorLeaderCAN = 0;
    public static final int kElevatorFollowerCAN = 1;

    public static final double kGearRatio = 7.5;
    public static final double kCarriageMass = Units.lbsToKilograms(3.8);
    public static final double kDrumRadius = Units.inchesToMeters(1.625);

    public static final double kElevatorHeightMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(39.75);

    public static final double kMaxVelocity = 1.70; // m/s
    public static final double kMaxAcceleration = 1.0; // TODO: modify

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxV = 10.0; // volts
    public static final double kS = 0.0; // negligible
    public static final double kG = 0.23;
    public static final double kV = 6.85;
    public static final double kA = 0.04;
    // public static final double kT = 18.17; // motor torque constant (mNm/A)

    public static final int kEncoderCPR = 2048;
    // distance per pulse = (distance per revolution) / (pulses per revolution) =
    // (Pi * D) / ppr

    // Elevator setpoints
    public static final double kLevel1 = Units.inchesToMeters(31.875);
    public static final double kLevel2 = Units.inchesToMeters(47.625);
    public static final double kLevel3 = Units.inchesToMeters(72.0);

    public static final double kElevatorDistPerPulse = (2 * Math.PI * kDrumRadius) / (kEncoderCPR / 4);

  }
}
