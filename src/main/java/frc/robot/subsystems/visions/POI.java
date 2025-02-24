/**
 * By shueja, FRC 6995
 */
package frc.robot.subsystems.visions;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.AllianceFlipUtil;

public class POI {
    public static final POI REEF_CENTER = new POI(
        new Pose2d(4.491799831390381, 4.026460647583008, Rotation2d.kZero)
    );

    private Supplier<Pose2d> flippedPose;
    public POI(Pose2d bluePose) {
        this.flippedPose = AllianceFlipUtil.getFlipped(bluePose);
    }
    public Pose2d flipped() {
        return flippedPose.get();
    }
}
