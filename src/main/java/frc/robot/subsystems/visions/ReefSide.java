/**
 * By shueja, FRC 6995
 */
package frc.robot.subsystems.visions;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

class ReefSideConstants {
    public static final double kXOffsetDistanceMeters = Units.inchesToMeters(5) + // gap from reef to bumper
            Units.inchesToMeters(3.25 + 14); // bumper width + half robot frame size
    // tagPose +X is straight out from the tag, +Y is to the left from that
    // perspective, so to the right from facing the reef
    // leftTransform needs +X, -Y, rightTransform needs +X, +Y
    // Front of robot faces reef, so need a 180 degree turn
    public static final Transform2d leftTransform = new Transform2d(
            kXOffsetDistanceMeters, VisionConstants.kTagMiddleToLeftEdge, Rotation2d.k180deg); //the transformation towards the left side of the april tag
    public static final Transform2d rightTransform = new Transform2d(
            kXOffsetDistanceMeters, VisionConstants.kTagMiddleToRightEdge, Rotation2d.k180deg); //the transformation towards the right side of the april tag
}

public enum ReefSide {
    R1(18), 
    R2(17),
    R3(22),
    R4(21),
    R5(20),
    R6(19); // Put these in whatever order matches an R1-6 mapping that makes sense for you.

    public final POI left; //left point of intersection object
    public final POI right; //right point of intersection object

    private ReefSide(POI left, POI right) {
        this.left = left;
        this.right = right;
    }

    public POI left() {return this.left;}
    public POI right() {return this.right;}    

    private ReefSide(int blueTagID) {
        AprilTagFieldLayout layout = VisionConstants.aprilTagFieldLayout; // Make apriltagFieldLayout public static
                                                                          // final
        Pose2d tagPose = layout.getTagPose(blueTagID).get().toPose2d(); // WARNING: no safety for invalid tags, just
                                                                        // don't do that above.
        // tagPose +X is straight out from the tag, +Y is to the left from that
        // perspective, so to the right from facing the reef
        left = new POI(tagPose.plus(ReefSideConstants.leftTransform));
        right = new POI(tagPose.plus(ReefSideConstants.rightTransform));
    }
}