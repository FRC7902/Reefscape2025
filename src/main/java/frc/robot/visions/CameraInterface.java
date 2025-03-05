package frc.robot.visions;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CameraInterface {
    public PhotonCamera camera;
    private PhotonPoseEstimator photonEstimator; //creates pose estimator object
    private boolean targetIsVisible = false;
    private double targetYaw = 0;
    private Pose2d poseOfAprilTag = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d poseFromRobotToTag = new Pose2d(0, 0, new Rotation2d(0));
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);


    public CameraInterface(String cameraName) {
        camera = new PhotonCamera(cameraName);
        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.krobotToCam);
    }

    private boolean isReefAprilTag(int aprilTagID) {
        switch (aprilTagID) {
            case -1: return false;
            case 1: return false;
            case 2: return false;
            case 3: return false;
            case 4: return false;
            case 5: return false;
            case 12: return false;
            case 13: return false;
            case 14: return false;
            case 15: return false;
            case 16: return false;
            default: return true;
        }
    }

    public boolean cameraSawTarget() {
        return targetIsVisible;
    }

    public Pose2d getRobotToTagPose() {
        return poseFromRobotToTag;
    }

    public double getYaw() {
        return targetYaw;
    }

    public void resetTargetDetector() {
        targetIsVisible = false;
    }

    public Pose2d getRobotToTagPose(Pose2d pose) {
        Pose2d robotPose = RobotContainer.drivebase.getPose();
        return new Pose2d(pose.getX() - robotPose.getX(), pose.getY() - robotPose.getY(), pose.getRotation());
    }

    public void getCameraResults() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getArea() >= 0.75) {
                        int targetID = target.getFiducialId();
                        if (isReefAprilTag(targetID)) {
                            targetYaw = target.getYaw();
                            poseOfAprilTag = aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();
                            poseFromRobotToTag = getRobotToTagPose(poseOfAprilTag);
                            targetIsVisible = true;
                            break;
                        }
                    }
                }
            }
        }
    }
}
