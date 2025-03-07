package frc.robot.visions;

import java.util.List;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class CameraInterface {
    public final PhotonCamera camera;
    public boolean targetIsVisible = false;
    public double targetYaw = 0;
    public Pose2d poseOfAprilTag = new Pose2d(0, 0, new Rotation2d(0));
    public Pose2d poseFromRobotToTag = new Pose2d(0, 0, new Rotation2d(0));
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public int aprilTagID = -1;
    List<Waypoint> waypoints; 
    PathConstraints constraints;
    PathPlannerPath path;
    List<Pose2d> scannedAprilTagPoses;
    

     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName) {
        camera = new PhotonCamera(cameraName);
        constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    }

     /**
     * Determines whether the April Tag scanned by the camera is an April Tag on the reef (for auto-align).
     *
     * @param aprilTagID
     * 
     * @return Whether the April Tag detected by the camera is an April Tag on the reef or not.
     */ 
    public boolean isReefAprilTag(int aprilTagID) {
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

     /**
     * Determines if the camera saw a valid April Tag.
     *
     * @return Whether a valid April Tag was detected or not.
     */ 
    public boolean cameraSawTarget() {
        return targetIsVisible;
    }

     /**
     * Gets the distance from the April Tag to the robot.
     *
     * @return The distance from the April Tag to the robot in a Pose2d object.  
     */ 
    public Pose2d getRobotToTagPose() {
        return poseFromRobotToTag;
    }

     /**
     * Gets the rotation of the April Tag relative to the camera.
     *
     * @return The rotation of the April Tag relative to the camera (in degrees).
     */ 
    public double getYaw() {
        return targetYaw;
    }

    /**
     * Sets the targetIsVisible boolean to false which is used to determine if the camera saw a valid April Tag.
     *
     */ 
    public void resetTargetDetector() {
        targetIsVisible = false;
    }

    /**
     * Gets the robot's current position.
     *
     * @return The robot's current position in a Pose2d object.
     */       
    private Pose2d getRobotPose() {
        return RobotContainer.drivebase.getPose();
    }

    /**
     * Clears the camera's FIFO buffer which contains up to 20 April Tags the camera has detected. This is done to ensure the latest April Tags are only seen for auto-aligning.
     *
     */     
    public void clearCameraFIFOBuffer() {
        camera.getAllUnreadResults();
    }

     /**
     * Calculates the distance from the target April Tag to the robot.
     *
     * @param aprilTagPose 
     */ 
    public Pose2d getRobotToTagPose(Pose2d aprilTagPose) {
        final Pose2d robotPose = getRobotPose();
        return new Pose2d(aprilTagPose.getX() - robotPose.getX(), aprilTagPose.getY() - robotPose.getY(), aprilTagPose.getRotation());
    }

    public double getRobotTranslationDistance() {
        final Translation2d robotTranslation = RobotContainer.drivebase.getSwerveDrive().getPose().getTranslation();
        final Translation2d aprilTagTranslation = poseOfAprilTag.getTranslation();
        return robotTranslation.getDistance(aprilTagTranslation);
    }

    public double getDistance() {
        return Math.sqrt((Math.pow(getRobotToTagPose().getX(), 2) + Math.pow(getRobotToTagPose().getY(), 2)));
    }

    public void setTagWayPoint() {
        PathPlannerPath.waypointsFromPoses(poseOfAprilTag);
    }


  
    public void createNewPathPlannerPath() {
        path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );        
    }

// The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
   

     /**
     * Uses the camera to determine if there is an April Tag in it's FOV.
     * If an april tag is detected, the "targetIsVisible" boolean is set to true, which is necessary for any commands related to the CameraInterface to ensure no null values are accidentally read (will throw NullPointerException if read).
     */     

    public Pose2d getClosestAprilTagPose() {
        return RobotContainer.drivebase.getPose().nearest(scannedAprilTagPoses);
    }
    

    public Twist2d getDistanceFromRobotToTag(Pose2d aprilTagPose) {
        return RobotContainer.drivebase.getPose().log(aprilTagPose);
    }

    public int getTargetAprilTagID() {
        return aprilTagID;
    }


    public void getCameraResults(int gooner) {
        final var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (final var target : result.getTargets()) {
                    if (target.getArea() >= VisionConstants.kAprilTagArea) {
                        final int targetID = target.getFiducialId();
                        if (isReefAprilTag(targetID)) {
                            targetYaw = target.getYaw();
                            poseOfAprilTag = aprilTagFieldLayout.getTagPose(targetID).get().toPose2d();
                            scannedAprilTagPoses.add(poseOfAprilTag);
                            poseFromRobotToTag = getRobotToTagPose(poseOfAprilTag);
                            targetIsVisible = true;
                            aprilTagID = targetID;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    public void getCameraResults() {
        poseOfAprilTag = aprilTagFieldLayout.getTagPose(18).get().toPose2d();
        poseFromRobotToTag = getRobotToTagPose(poseOfAprilTag);
        targetIsVisible = true;
        targetYaw = 0;
    }
}
