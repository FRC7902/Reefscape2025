package frc.robot.visions;

import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class CameraInterface extends SubsystemBase {
    private final PhotonCamera camera;
    private boolean targetIsVisible = false;
    private double targetYaw = 0;
    private int aprilTagID = -1;
    private double aprilTagArea = 0;
    private final double aprilTagAreaLimit;
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private VisionSystemSim visionSim;
    private AprilTagFieldLayout tagLayout;

    private double yTranslation;

    private NetworkTable m_networkTable;



     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName, double aprilTagAreaLimit) {
        camera = new PhotonCamera(cameraName);
        this.aprilTagAreaLimit = aprilTagAreaLimit;
          
        SmartDashboard.putNumber("kPY Close", VisionConstants.kPY2);
        SmartDashboard.putNumber("kIY Close", VisionConstants.kIY2);
        SmartDashboard.putNumber("kDY Close", VisionConstants.kDY2);

        SmartDashboard.putNumber("kPY Far", VisionConstants.kPY);
        SmartDashboard.putNumber("kIY Far", VisionConstants.kIY);
        SmartDashboard.putNumber("kDY Far", VisionConstants.kDY);

        SmartDashboard.putNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);
        visionSim = new VisionSystemSim("main");
        TargetModel targetModel = new TargetModel(0.5, 0.25);
        Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

        visionSim.addVisionTargets(visionTarget);

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            System.out.println("YAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        }
        catch (Exception e) {
            System.out.println("TUAHTUAHTUAHTUAHTUAHTUAHTUATHUATHAJDKHAKSDHJKASDHKJASHDJKASHDJKAHSDKJAHSDKJAHSDKJHASJDKHAWDUIWAIUDASHK");
        }

        visionSim.addAprilTags(tagLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1920, 1080, Rotation2d.fromDegrees(1));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);

        visionSim.getDebugField();

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        cameraSim.enableDrawWireframe(true);

        m_networkTable = NetworkTableInstance.getDefault().getTable("photonvision");
    }

     /**
     * Determines whether the April Tag scanned by the camera is an April Tag on the reef (for auto-align).
     * 
     * @return Whether the April Tag detected by the camera is an April Tag on the reef or not.
     */ 
    public boolean isReefAprilTag() {
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

    public double getAprilTagYTranslation() {
        return yTranslation;
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
                case 17: return -1.047; //60 degrees
                case 18: return 0; //0 degrees
                case 19: return -5.236; //300 degrees
                case 20: return -4.189; //240 degrees
                case 21: return 3.142; //180 degrees
                case 22: return -2.094; //120 degrees
            }
        }

        else if (alliance.get() == Alliance.Red) {
            switch (aprilTagID) {
                case 6: return -5.236; //300 degrees //check
                case 7: return 0; //0 degrees
                case 8: return -1.047; //60 degrees
                case 9: return -2.094; //120 degrees
                case 10: return 3.142; //180 degrees
                case 11: return -4.189; //240 degrees
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
                    aprilTagArea = target.getArea();
                    if (aprilTagArea >= aprilTagAreaLimit) {
                        aprilTagID = target.getFiducialId();
                        if (isReefAprilTag()) {
                            targetYaw = target.getYaw();
                            targetIsVisible = true;
                            yTranslation = m_networkTable.getEntry("targetPixelsY").getDouble(yTranslation);
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
        SmartDashboard.putNumber("April Tag Y Translation", getAprilTagYTranslation());
        SmartDashboard.putNumber("April Tag ID", getTargetAprilTagID());
        SmartDashboard.putNumber("April Tag Rotation", getAprilTagRotation());
        SmartDashboard.putNumber("April Tag Area", aprilTagArea);

        VisionConstants.kPY2 = SmartDashboard.getNumber("kPY Close", VisionConstants.kPY2);
        VisionConstants.kIY2 = SmartDashboard.getNumber("kIY Close", VisionConstants.kIY2);
        VisionConstants.kDY2 = SmartDashboard.getNumber("kDY Close", VisionConstants.kDY2);

        VisionConstants.kPY = SmartDashboard.getNumber("kPY Far", VisionConstants.kPY);
        VisionConstants.kIY = SmartDashboard.getNumber("kIY Far", VisionConstants.kIY);
        VisionConstants.kDY = SmartDashboard.getNumber("kDY Far", VisionConstants.kDY);

        VisionConstants.yControllerTolerance = SmartDashboard.getNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(RobotContainer.m_swerveSubsystem.getPose());
    }
}