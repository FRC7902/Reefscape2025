package frc.robot.visions;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class CameraInterface extends SubsystemBase {
    private final PhotonCamera camera;
    private boolean targetIsVisible = false;
    private double targetYaw = 0;
    private int aprilTagID = -1;
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    

     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName) {
        camera = new PhotonCamera(cameraName);
          
        SmartDashboard.putNumber("kPY Close", VisionConstants.kPY2);
        SmartDashboard.putNumber("kIY Close", VisionConstants.kIY2);
        SmartDashboard.putNumber("kDY Close", VisionConstants.kDY2);

        SmartDashboard.putNumber("kPY Far", VisionConstants.kPY);
        SmartDashboard.putNumber("kIY Far", VisionConstants.kIY);
        SmartDashboard.putNumber("kDY Far", VisionConstants.kDY);

        SmartDashboard.putNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);
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
     * Gets the rotation of the April Tag relative to the camera.
     *
     * @return The rotation of the April Tag relative to the camera (in degrees).
     */ 
    public double getAprilTagYaw() {
        return targetYaw;
    }

    /**
     * Gets the rotation of the April Tag relative to the field.
     *
     * @return The rotation of the April Tag relative to the field (in radians).
     */      
    public double getAprilTagRotation() {
        final int aprilTagID = getTargetAprilTagID();
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get() == Alliance.Blue) {
            switch (aprilTagID) {
                case 17: return 1.047; //60 degrees
                case 18: return 0; //0 degrees
                case 19: return 5.236; //300 degrees
                case 20: return 4.189; //240 degrees
                case 21: return 0; //180 degrees
                case 22: return 2.094; //120 degrees
            }
        }

        else if (alliance.get() == Alliance.Red) {
            switch (aprilTagID) {
                case 6: return 5.236; //300 degrees
                case 7: return 0; //0 degrees
                case 8: return 1.047; //60 degrees
                case 9: return 2.094; //120 degrees
                case 10: return 3.142; //180 degrees
                case 11: return 4.189; //240 degrees
            }
        }
        return 0;
    }


    /**
     * Sets the targetIsVisible boolean to false which is used to determine if the camera saw a valid April Tag.
     *
     */ 
    public void resetTargetDetector() {
        targetIsVisible = false;
    } 

    /**
     * Gets the ID of the April Tag that is detected by the camera.
     *
     * @return The ID of the April Tag.
     */    
    public int getTargetAprilTagID() {
        return aprilTagID;
    }

    /**
     * Checks the camera to see whether an April Tag is in view of the camera. This is used to determine whether the auto-align command should run or not.
     *
     * @return Whether an April Tag is in view of the camera or not.
     */        
    public boolean cameraHasSeenAprilTag() {
        resetTargetDetector(); //resets target detector so that we don't get old results 
        getCameraResults(); //gets results from camera
        return cameraSawTarget(); //returns whether the camera has seen an april tag or not
    }

    /**
     * Runs the camera to check for an April Tag. If an April Tag is in sight, the method gathers the yaw and the ID of the April Tag.
     *
     */   
    public void getCameraResults() {
        final var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (final var target : result.getTargets()) {
                    if (target.getArea() >= VisionConstants.kAprilTagAreaLimit) {
                        final int targetID = target.getFiducialId();
                        if (isReefAprilTag(targetID)) {
                            targetYaw = target.getYaw();
                            targetIsVisible = true;
                            break;
                        }
                    }
                }
            }
        }
    } 

    @Override
    public void periodic() {

        SmartDashboard.putNumber("April Tag Yaw", getAprilTagYaw());
    
        VisionConstants.kPY2 = SmartDashboard.getNumber("kPY Close", VisionConstants.kPY2);
        VisionConstants.kIY2 = SmartDashboard.getNumber("kIY Close", VisionConstants.kIY2);
        VisionConstants.kDY2 = SmartDashboard.getNumber("kDY Close", VisionConstants.kDY2);

        VisionConstants.kPY = SmartDashboard.getNumber("kPY Far", VisionConstants.kPY);
        VisionConstants.kIY = SmartDashboard.getNumber("kIY Far", VisionConstants.kIY);
        VisionConstants.kDY = SmartDashboard.getNumber("kDY Far", VisionConstants.kDY);

        VisionConstants.yControllerTolerance = SmartDashboard.getNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);
    }

}