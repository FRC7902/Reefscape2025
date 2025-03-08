package frc.robot.visions;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class CameraInterface extends SubsystemBase {
    public final PhotonCamera camera;
    public boolean targetIsVisible = false;
    public double targetYaw = 0;
    public Pose2d poseOfAprilTag = new Pose2d(0, 0, new Rotation2d(0));
    public Pose2d poseFromRobotToTag = new Pose2d(0, 0, new Rotation2d(0));
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public int aprilTagID = -1;
    List<Pose2d> scannedAprilTagPoses;
    

    private final VisionSystemSim visionSim;
    private TargetModel targetModel;
    private SimCameraProperties m_simCamProperties;
    private PhotonCameraSim s_autoAlignCam;

    private double targetRange = 0;

    private double cameraOffsetToRobot = 0;


     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName, double cameraOffsetToRobot) {
        camera = new PhotonCamera(cameraName);
        visionSim = new VisionSystemSim("main");
        targetModel = TargetModel.kAprilTag36h11;
        Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        // The given target model at the given pose
        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

        this.cameraOffsetToRobot = cameraOffsetToRobot;

        // Add this vision target to the vision system simulation to make it visible
        visionSim.addVisionTargets(visionTarget);
        visionSim.addAprilTags(aprilTagFieldLayout);

        m_simCamProperties = new SimCameraProperties();

        m_simCamProperties.setCalibration(640, 480,Rotation2d.fromDegrees(0));
        m_simCamProperties.setCalibError(0.25, 0.08);
        m_simCamProperties.setFPS(60);
        m_simCamProperties.setAvgLatencyMs(25);
        m_simCamProperties.setLatencyStdDevMs(5);

        s_autoAlignCam = new PhotonCameraSim(camera, m_simCamProperties);
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        visionSim.addCamera(s_autoAlignCam, robotToCamera);
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

    public Pose2d getClosestAprilTagPose() {
        return RobotContainer.drivebase.getPose().nearest(scannedAprilTagPoses);
    }

    public int getTargetAprilTagID() {
        return aprilTagID;
    }

    public double getTargetRange() {
        return targetRange;
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
                            targetRange = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kGroundToCameraDistance, 1.435, Units.degreesToRadians(VisionConstants.kCameraPitch), Units.degreesToRadians(target.getPitch()));
                            targetIsVisible = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    public void getCameraResults(int sigma) {
        poseOfAprilTag = aprilTagFieldLayout.getTagPose(18).get().toPose2d();
        poseFromRobotToTag = getRobotToTagPose(poseOfAprilTag);
        targetIsVisible = true;
        targetYaw = 0;
    }


    @Override
    public void periodic() {
        VisionConstants.kPY = SmartDashboard.getNumber("kP Y", 0.03);
        VisionConstants.kIY = SmartDashboard.getNumber("kI Y", 0);
        VisionConstants.kDY = SmartDashboard.getNumber("kD Y", 0.01);

        VisionConstants.kPOmega = SmartDashboard.getNumber("kP Omega", 0.03);
        VisionConstants.kIOmega = SmartDashboard.getNumber("kI Omega", 0);
        VisionConstants.kDOmega = SmartDashboard.getNumber("kD Omega", 0.01);

    }

    @Override
    public void simulationPeriodic() {
        visionSim.getDebugField();
        visionSim.update(getRobotPose());
    }

}
