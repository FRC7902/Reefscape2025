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

  public static class ClimbConstants {
    //CAN ID of the primary motor
    public static final int kClimbLeaderMotorCANID = 0;
    //CAN ID of the secondary motor
    public static final int kClimbFollowerMotorCANID = 1;
    //current limit when motor rpm is at 0 (in amps)
    public static final int kMotorStallCurrent = 0;
    //current limit when motor rpm is greater than 5700 rpm (since we are using neo motors) (in amps)
    public static final int kMotorFreeSpeedCurrent = 0;
    //rpm limit, where any rpm values that are below the limit you set will be set to the stall current.
    //rpm greater than the set rpm limit will linearly increase to the free current limit.
    public static final int kMotorRPMLimit = 0;
    //speed of motors when moving up
    public static final double kClimbUpMotorSpeed = 0.5;
    //speed of motors when moving down

    public static final double kClimbRestPosition = 0.5;
    public static final double kClimbRaisedPosition = 100;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

  }
}
