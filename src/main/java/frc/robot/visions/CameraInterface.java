package frc.robot.visions;

import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class CameraInterface {
    public final PhotonCamera camera;
    public boolean targetIsVisible = false;
    public double targetYaw = 0;
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public int aprilTagID = -1;
    
    private double cameraOffsetToRobot = 0;


     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName, double cameraOffsetToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraOffsetToRobot = cameraOffsetToRobot;
    }

     /**
     * Determines whether the April Tag scanned by the camera is an April Tag on the reef (for auto-align).
     *
     * @param aprilTagID
     * 
     * @return Whether the April Tag detected by the camera is an April Tag on the reef or not.
     */ 

    public double getCameraOffset() {
        return cameraOffsetToRobot;
    }

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
     * Sets the targetIsVisible boolean to false which is used to determine if the camera saw a valid April Tag.
     *
     */ 
    public void resetTargetDetector() {
        targetIsVisible = false;
    } 

    public int getTargetAprilTagID() {
        return aprilTagID;
    }

    public double getAprilTagRotation() {
        int aprilTagID = getTargetAprilTagID();
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get() == Alliance.Blue) {
            switch (aprilTagID) {
                case 17: return 1.047; //60 degrees
                case 18: return 0; //0 degrees
                case 19: return 5.236; //300 degrees
                case 20: return 4.189; //240 degrees
                case 21: return 3.142; //180 degrees
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

    public boolean cameraHasSeenAprilTag() {
        resetTargetDetector();
        getCameraResults();
        return cameraSawTarget();
    }

    public void getCameraResults() {
        final var results = camera.getAllUnreadResults();
        //System.out.println("sigma");
        if (!results.isEmpty()) {
            //System.out.println("YAAAAAAA");
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                //System.out.println("WE GOT APRIL TAGSSS");
                for (final var target : result.getTargets()) {
                    if (target.getArea() >= VisionConstants.kAprilTagArea) {
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

}