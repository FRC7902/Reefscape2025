package frc.robot.subsystems.visions;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class Camera {

    private final PhotonCamera camera; //creates camera object
    private final PhotonPoseEstimator photonEstimator; //creates pose estimator object
    private Matrix<N3, N1> curStdDevs; //creates matrix for current standard deviations


    public Camera(String cameraName) {
       this.camera = new PhotonCamera(cameraName); 
       //creates a new camera object which is created in RobotContainer
       photonEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam);
       //creates a PhotonPoseEstimator object which fuses the camera odometry and estimates the robot's position based on the april tag field layout
       //the camera will combine the poses of the april tags it detects into one pose estimate
       //an offset must be set if the camera is not centered
       photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
       //if the camera only detects one april tag, it will use the april tag that has the lowest ambiguity
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        //creates a new container object that can hold null values and non-null values
        for (var change : camera.getAllUnreadResults()) {
            //gets the latest results from the camera
            visionEst = photonEstimator.update(change);
            //updates vision estimates. will not update if no april tags have been detected
            updateEstimationStdDevs(visionEst, change.getTargets());
            //updates the estimated standard deviations with the results
        }
        return visionEst;
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                //gets the raw data from the camera
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                //obtains the pose value from the april tag that is detected by the camera
                if (tagPose.isEmpty()) continue; //skips current iteration if there is no target detected
                numTags++; //counts the number of tags the camera saw
                //adds the average distance from the robot to the april tag
                //to be used to calculate the average distance
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                //find saverage distance from robot to camera
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }  

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

}
