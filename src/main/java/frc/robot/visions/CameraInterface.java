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
          
        SmartDashboard.putNumber("kPY Close", VisionConstants.kPY2);
        SmartDashboard.putNumber("kIY Close", VisionConstants.kIY2);
        SmartDashboard.putNumber("kDY Close", VisionConstants.kDY2);

        SmartDashboard.putNumber("kPY Far", VisionConstants.kPY);
        SmartDashboard.putNumber("kIY Far", VisionConstants.kIY);
        SmartDashboard.putNumber("kDY Far", VisionConstants.kDY);

        SmartDashboard.putNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);

        LimelightHelpers.SetFiducialIDFiltersOverride(camera, VisionConstants.acceptedTagIDs); // Only track these tag IDs

        LimelightHelpers.setLEDMode_PipelineControl(camera);

        LimelightHelpers.setCameraPose_RobotSpace(
            camera, 
            VisionConstants.kFowardToCamera,    // Forward offset (meters)
            VisionConstants.kSidewaysToCamera,    // Side offset (meters)
            VisionConstants.kGroundToCamera,    // Height offset (meters)
            0.0,    // Roll (degrees)
            0.0,   // Pitch (degrees)
            VisionConstants.kCameraRotation     // Yaw (degrees)
        );

        LimelightHelpers.SetIMUMode(camera, 2);
        LimelightHelpers.setStreamMode_Standard(camera);
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

    public void resetLimelightIMU() {
        LimelightHelpers.SetIMUMode(camera, 0);
        LimelightHelpers.SetIMUMode(camera, 2);
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


    @Override
    public void periodic() {

        VisionConstants.kPY2 = SmartDashboard.getNumber("kPY Close", VisionConstants.kPY2);
        VisionConstants.kIY2 = SmartDashboard.getNumber("kIY Close", VisionConstants.kIY2);
        VisionConstants.kDY2 = SmartDashboard.getNumber("kDY Close", VisionConstants.kDY2);

        VisionConstants.kPY = SmartDashboard.getNumber("kPY Far", VisionConstants.kPY);
        VisionConstants.kIY = SmartDashboard.getNumber("kIY Far", VisionConstants.kIY);
        VisionConstants.kDY = SmartDashboard.getNumber("kDY Far", VisionConstants.kDY);

        VisionConstants.yControllerTolerance = SmartDashboard.getNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);

        double robotYaw = RobotContainer.m_swerveSubsystem.getSwerveDrive().getYaw().getDegrees();  

        // LimelightHelpers.SetRobotOrientation(camera, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // // Get the pose estimate
        // LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);

        // double rotationSpeed = Math.abs(RobotContainer.m_swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond);
        // boolean shouldRejectUpdate = rotationSpeed < 6.28319 && getAprilTagArea() > 50; //360 degrees

        // if (!(shouldRejectUpdate)) {
        //     RobotContainer.m_swerveSubsystem.getSwerveDrive().addVisionMeasurement(
        //         limelightMeasurement.pose,
        //         limelightMeasurement.timestampSeconds,
        //         VecBuilder.fill(.5, .5, 9999999));  
        // }

    }
}