// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.*;

/** Add your docs here. */
public class FirebirdUtils {

    public double metersToRotations(double meters) {
        return meters / (2 * Math.PI * Constants.ElevatorConstants.kDrumRadius);
    }

    public double rotationsToMeters(double rotations) {
        return rotations * (2 * Math.PI * Constants.ElevatorConstants.kDrumRadius);
    }
}