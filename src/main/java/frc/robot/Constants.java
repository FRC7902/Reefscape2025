// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class ElevatorConstants {
    public static final int kElevatorLeaderCAN = 0;
    public static final int kElevatorFollowerCAN = 1;

    public static final double kElevatorkP = 0.0;
    public static final double kElevatorkI = 0.0;
    public static final double kElevatorkD = 0.0;

    public static final double kGearRatio = 0.0;
    public static final double kCarriageMass = 0.0;
    public static final double kDrumRadius = 0.0;

    public static final double kElevatorHeightMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 0.0;

    public static final double kElevatorkS = 0.0;
    public static final double kElevatorkG = 0.0;
    public static final double kElevatorkV = 0.0;
    public static final double kElevatorkA = 0.0;
    public static final double kT = 18.17;  // mNm/A

    public static final int kEncoderCPR = 2048;
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorDistPerPulse = 0.0;

  }
}
