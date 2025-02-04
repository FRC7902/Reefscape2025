// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
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

    public static final double kMaxVelocity = 1.70;
    public static final double kMaxAcceleration = 1.0;  // TODO: modify

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kMaxV = 10.0; // volts 
    public static final double kS = 0.0; // negligible
    public static final double kG = 0.23;
    public static final double kV = 6.85;
    public static final double kA = 0.04;

    // Elevator setpoints
    public static final double kLevel1 = Units.inchesToMeters(31.875);
    public static final double kLevel2 = Units.inchesToMeters(47.625);
    public static final double kLevel3 = Units.inchesToMeters(72.0);

    public static final double kTargetError = 2;

    public static final int kEncoderCPR = 2048;
    public static final double kElevatorDistPerPulse = (2 * Math.PI * kDrumRadius) / (kEncoderCPR / 4);

  }
}
