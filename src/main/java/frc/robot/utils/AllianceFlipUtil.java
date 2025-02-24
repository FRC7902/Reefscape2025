// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class AllianceFlipUtil {
  public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
  public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

  public static double flipX(double x) {
    return fieldLength - x;
  }

  public static double applyX(double x) {
    return shouldFlip() ? flipX(x) : x;
  }

  public static double flipY(double y) {
    return fieldWidth - y;
  }

  public static double applyY(double y) {
    return shouldFlip() ? flipY(y) : y;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? flip(rotation) : rotation;
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? flip(pose) : pose;
  }

  public static Supplier<Pose2d> getFlipped(Pose2d pose) {
    Pose2d flipped = flip(pose);
    return () -> shouldFlip() ? flipped : pose;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}