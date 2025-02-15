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
  public static final class IOConstants {
    public static final int kDriverStick = 0;
    public static final int kOperatorStick = 1;
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int motorCurrentLimit=40; 
    public static final int intakeVoltage=7;
    public static final int outtakeVoltage=-7;
    public static final int elevatorManipulatorForward=7;
    public static final int elevatorManipulatorReverse=-7;
    public static final int beamBreakPort=-1;
    public static final int elevatorManipulatorPort=7;//idk
    
    public static final int FullyUpArmAngleSetpoint=80;
    public static final int FullyDownArmAngleSetpoint=0;
    public static final int groundIntakeRollerID=0;
    public static final int elevatorManipulatorID=1;
    public static final double algaeArmControllerKp=0.01;
    public static final double algaeArmControllerKi=0.01;
    public static final double algaeArmControllerKd=0.01;
    public static final int armPivotEncoderPin=1;
    public static final int groundIntakePivotLimit=20;
    public static final int armUpVoltage=7;
    public static final double armDownVoltage=-7;

  }

  public static class IntakeConstants {
    public static final int intakeCANid = 8;
    public static final double suckingSpeed = 1;
    public static final double outtakingSpeed = -1;
    public static final double feedingSpeed = 1;
    public static final double holdPower = 0;
    public static final int beamBrakePort = 5;
    public static final double kSFeedForward = 0;
    public static final double kVFeedForward = 0;
    public static final double kAFeedForward = 0;
    public static final int intakeCurrentLimit = 40;
    public static final double intakeTargetSpeed = 1;
    public static final double pullingSpeed = -0.1;
  }

  }
