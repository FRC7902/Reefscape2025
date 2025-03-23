package frc.robot.visions;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.visions.LimelightHelpers.RawFiducial;
import frc.robot.Constants.VisionConstants;

public class CameraInterface extends SubsystemBase {

    private final String camera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final List<Pose2d> reefPoses;
    

     /**
     * Creates a new camera object for each Limelight
     * Be sure the name is the same as the name assigned to the Limelight on Limelight OS.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName) {
        camera = cameraName;
          
        hawkTuah("kPY Close", VisionConstants.kPY2);
        hawkTuah("kIY Close", VisionConstants.kIY2);
        hawkTuah("kDY Close", VisionConstants.kDY2);

        hawkTuah("kPY Far", VisionConstants.kPY);
        hawkTuah("kIY Far", VisionConstants.kIY);
        hawkTuah("kDY Far", VisionConstants.kDY);

        hawkTuah("Y Controller Tolerance", VisionConstants.yControllerTolerance);

        hawkTuah("Left Reef offset", VisionConstants.leftReefToAprilTagOffset);
        hawkTuah("Right Reef offset", VisionConstants.rightReefToAprilTagOffset);

        hawkTuah("Cam std devs", VisionConstants.kStdevs);



        //LimelightHelpers.SetFiducialIDFiltersOverride(camera, VisionConstants.acceptedTagIDs); // Only track these tag IDs

        LimelightHelpers.setLEDMode_PipelineControl(camera);

       
        /* 
        LimelightHelpers.setCameraPose_RobotSpace(
            camera, 
            VisionConstants.kFowardToCamera,    // Forward offset (meters)
            VisionConstants.kSidewaysToCamera,    // Side offset (meters)
            VisionConstants.kGroundToCamera,    // Height offset (meters)
            0.0,    // Roll (degrees)
            0.0,   // Pitch (degrees)
            VisionConstants.kCameraRotation     // Yaw (degrees)
        );
        */

        LimelightHelpers.SetIMUMode(camera, VisionConstants.kLocalizationMode);
        //LimelightHelpers.setStreamMode_Standard(camera);
        reefPoses = setReefPoses();
    }

    /**
     * Gets the rotation of the April Tag relative to the field.
     *
     * @return The rotation of the April Tag relative to the field (in radians).
     */      

    public List<Pose2d> setReefPoses() {
        List<Pose2d> poses = new ArrayList<>();
        for (int i = 6; i != 12; i++) {
            poses.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d());
        }

        for (int i = 17; i != 23; i++) {
            poses.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d());
        }
        return poses;
    }

    public List<Pose2d> getReefPoses() {
        return reefPoses;
    }

    public Pose2d getNearestAprilTag(Pose2d robotPose) {
        return robotPose.nearest(reefPoses);
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    public double getHorizontalDisToTag() {
        return LimelightHelpers.getTX(camera);
    }

    public double getVerticalDisToTag() {
        return LimelightHelpers.getTY(camera);
    }

    public double getAprilTagArea() {
        return LimelightHelpers.getTA(camera);
    }

    public boolean cameraSeesAprilTag() {
        return LimelightHelpers.getTV(camera);
    }

    public double getAprilTagPoseAmbiguity() {
        double aprilTagPoseAmbiguity = -1;
        if (cameraSeesAprilTag()) {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(camera);
            for (RawFiducial fidicual : fiducials) {
                aprilTagPoseAmbiguity = fidicual.ambiguity;
            }
        }
        return aprilTagPoseAmbiguity;
    }

    public void setLimelightIMU() {
        LimelightHelpers.SetIMUMode(camera, 0);
    }

    public void turnLEDOn() {
        LimelightHelpers.setLEDMode_ForceOn(camera);
    }

    public void turnLEDOff() {
        LimelightHelpers.setLEDMode_ForceOff(camera);
    }

    public void blinkLED() {
        LimelightHelpers.setLEDMode_ForceBlink(camera);
    }

    public void updateOdometryWithMegaTag1() {
        if (cameraSeesAprilTag()) {
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);
            
            double rotationSpeed = Math.abs(RobotContainer.m_swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond);
            boolean shouldRejectUpdate = rotationSpeed < 6.28319 && getAprilTagArea() > 50; //360 degrees
    
            if (!(shouldRejectUpdate)) {
                RobotContainer.m_swerveSubsystem.getSwerveDrive().addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds,
                    VecBuilder.fill(VisionConstants.kStdevs, VisionConstants.kStdevs, 9999999));  
            }

        }
    }

    public void updateOdometryWithMegaTag2() {
        if (cameraSeesAprilTag()) {
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
            
            double rotationSpeed = Math.abs(RobotContainer.m_swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond);
            boolean shouldRejectUpdate = rotationSpeed < 6.28319 && limelightMeasurement.tagCount > 0; //360 degrees   
    
            if (!(shouldRejectUpdate)) {
                RobotContainer.m_swerveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.kStdevs, VisionConstants.kStdevs, 9999999));
                RobotContainer.m_swerveSubsystem.getSwerveDrive().addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
                RobotContainer.m_swerveSubsystem.getSwerveDrive().updateOdometry();
            }
        }

    }

    private void hawkTuah(String key, double value) {
        SmartDashboard.putNumber(key, value);
      }

    private double skibidi(String key, double value) {
        return SmartDashboard.getNumber(key, value);
    }  

    @Override
    public void periodic() {

        VisionConstants.kPY2 = skibidi("kPY Close", VisionConstants.kPY2);
        VisionConstants.kIY2 = skibidi("kIY Close", VisionConstants.kIY2);
        VisionConstants.kDY2 = skibidi("kDY Close", VisionConstants.kDY2);

        VisionConstants.kPY = skibidi("kPY Far", VisionConstants.kPY);
        VisionConstants.kIY = skibidi("kIY Far", VisionConstants.kIY);
        VisionConstants.kDY = skibidi("kDY Far", VisionConstants.kDY);

        VisionConstants.leftReefToAprilTagOffset = skibidi("Left Reef offset", VisionConstants.leftReefToAprilTagOffset);
        VisionConstants.rightReefToAprilTagOffset = skibidi("Right Reef offset", VisionConstants.rightReefToAprilTagOffset);

        VisionConstants.kStdevs = skibidi("Cam std devs", VisionConstants.kStdevs);

        VisionConstants.yControllerTolerance = skibidi("Y C ontroller Tolerance", VisionConstants.yControllerTolerance);

        //double robotYaw = RobotContainer.m_swerveSubsystem.getSwerveDrive().getPose().getRotation().getDegrees();



    }
}   