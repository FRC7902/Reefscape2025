package frc.robot.visions;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    private double cornerAverage = 0;
    private final PhotonPoseEstimator photonEstimator; //creates pose estimator object
    private Matrix<N3, N1> curStdDevs; //creates matrix for current standard deviations


    Transform3d kRobotToCam;

    private double xTranslation;

    private NetworkTable m_networkTable;

    private List<Pose2d> detectedPoses;

    double FOV_X = 70.0;  // Degrees
    double FOV_Y = 55.0;  // Degrees
    int RES_X = 640;
    int RES_Y = 480;

     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName, double aprilTagAreaLimit, Transform3d kRobotToCam) {
        camera = new PhotonCamera(cameraName);
        this.aprilTagAreaLimit = aprilTagAreaLimit;
        this.kRobotToCam = kRobotToCam;
        
        detectedPoses = new ArrayList<>();

        SmartDashboard.putNumber("kPY Close", VisionConstants.kPY2);
        SmartDashboard.putNumber("kIY Close", VisionConstants.kIY2);
        SmartDashboard.putNumber("kDY Close", VisionConstants.kDY2);

        SmartDashboard.putNumber("kPY Far", VisionConstants.kPY);
        SmartDashboard.putNumber("kIY Far", VisionConstants.kIY);
        SmartDashboard.putNumber("kDY Far", VisionConstants.kDY);

        SmartDashboard.putNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);
        photonEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        //creates a PhotonPoseEstimator object which fuses the camera odometry and estimates the robot's position based on the april tag field layout
        //the camera will combine the poses of the april tags it detects into one pose estimate
        //an offset must be set if the camera is not centered
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        //detectedPoses.add(new Pose2d(0, 0, new Rotation2d(0)));
    }


    public Pose2d getClosestPose(Pose2d robotPose) {
        //return robotPose.nearest(detectedPoses);
        return aprilTagFieldLayout.getTagPose(18).get().toPose2d();
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

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
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
    

        /* 
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
        */
        
    

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

    public double getAprilTagxTranslation() {
        return xTranslation;
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
        detectedPoses.clear();
        
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
        /* 
        resetTargetDetector(); //resets target detector so that we don't get old results 
        getCameraResults(); //gets results from camera
        return cameraSawTarget(); //returns whether the camera has seen an april tag or not
        */
        return true;
    }

    public List<Pose2d> getDetectedPoses() {
        return detectedPoses;
    }

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
                            detectedPoses.add(aprilTagFieldLayout.getTagPose(aprilTagID).get().toPose2d());
                            targetIsVisible = true;
                            //xTranslation = target.getBestCameraToTarget().getY();
                            //cornerAverage = getCentroid(target.getDetectedCorners());

                            //int x_pixel = (int) (((targetYaw + (FOV_X / 2)) / FOV_X) * RES_X);

                            SmartDashboard.putNumber("TUAH TO THE HAWK TO THE TUAH", xTranslation);
                            //yTranslation = m_networkTable.getEntry("targetPixelsY").getDouble(yTranslation);

                            break;
                        }
                    }
                }
            }
        }
    } 

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }


    @Override
    public void periodic() {

        VisionConstants.kPY2 = SmartDashboard.getNumber("kPY Close", VisionConstants.kPY2);
        VisionConstants.kIY2 = SmartDashboard.getNumber("kIY Close", VisionConstants.kIY2);
        VisionConstants.kDY2 = SmartDashboard.getNumber("kDY Close", VisionConstants.kDY2);

        VisionConstants.kPY = SmartDashboard.getNumber("kPY Far", VisionConstants.kPY);
        VisionConstants.kIY = SmartDashboard.getNumber("kIY Far", VisionConstants.kIY);
        VisionConstants.kDY = SmartDashboard.getNumber("kDY Far", VisionConstants.kDY);

        VisionConstants.yControllerTolerance = SmartDashboard.getNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);

        getCameraResults();
    }

    @Override
    public void simulationPeriodic() {
        //visionSim.update(RobotContainer.m_swerveSubsystem.getPose());
    }
}