package frc.robot.visions;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class CameraInterface {
    private PhotonCamera camera;
    private PhotonPoseEstimator photonEstimator; //creates pose estimator object
    private Matrix<N3, N1> curStdDevs; //creates matrix for current standard deviations
    private boolean targetIsVisible = false;
    public double targetYaw = 0;
    public double distanceToTag = 0;
    public double horizontalDistanceToTag = 0;
    
    public CameraInterface(String cameraName) {
        camera = new PhotonCamera(cameraName);
        photonEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.krobotToCam);
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

    public void resetTargetDetector() {
        targetIsVisible = false;
    }

    public void getCameraResults() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (isReefAprilTag(target.getFiducialId())) {
                        targetYaw = target.getYaw();
                        distanceToTag = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kGroundToCameraDistance, VisionConstants.kGroundToAprilTagDistance, Units.degreesToRadians(VisionConstants.kCameraPitch), Units.degreesToRadians(target.getPitch()));
                        horizontalDistanceToTag = distanceToTag * Math.sin(targetYaw);
                        targetIsVisible = true;
                        break;
                    }
                }
            }
        }
    }
}
