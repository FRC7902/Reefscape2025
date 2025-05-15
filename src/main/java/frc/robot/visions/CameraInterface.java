package frc.robot.visions;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.visions.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
import frc.robot.Constants.VisionConstants;

public class CameraInterface extends SubsystemBase {

    private final String camera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded);
    private final List<Pose2d> reefPoses;
    private final SwerveDrive m_swerveDrive;

    private double distanceToAprilTag = 0;
    private double poseAmbiguity;
    private int detectedID = 0;

    /**
     * Creates a new camera object for each Limelight
     * Be sure the name is the same as the name assigned to the Limelight on
     * Limelight OS.
     * 
     * @param cameraName
     */
    public CameraInterface(String cameraName, SwerveSubsystem m_swerveDrive) {
        camera = cameraName;
        this.m_swerveDrive = m_swerveDrive.getSwerveDrive();

        SmartDashboard.putNumber("kPY Close", VisionConstants.kPY2);
        SmartDashboard.putNumber("kIY Close", VisionConstants.kIY2);
        SmartDashboard.putNumber("kDY Close", VisionConstants.kDY2);

        SmartDashboard.putNumber("kPY Far", VisionConstants.kPY);
        SmartDashboard.putNumber("kIY Far", VisionConstants.kIY);
        SmartDashboard.putNumber("kDY Far", VisionConstants.kDY);

        SmartDashboard.putNumber("Y Controller Tolerance", VisionConstants.yControllerTolerance);

        SmartDashboard.putNumber("Left Reef offset", VisionConstants.leftReefToAprilTagOffset);
        SmartDashboard.putNumber("Right Reef offset", VisionConstants.rightReefToAprilTagOffset);

        SmartDashboard.putNumber("April tag limit", VisionConstants.kLocalizationDisLim);

        SmartDashboard.putNumber("STD DEVS", VisionConstants.kStdDevs);

        // LimelightHelpers.SetFiducialIDFiltersOverride(camera,
        // VisionConstants.acceptedTagIDs); // Only track these tag IDs

        LimelightHelpers.setLEDMode_PipelineControl(camera);

        /*
         * LimelightHelpers.setCameraPose_RobotSpace(
         * camera,
         * VisionConstants.kFowardToCamera, // Forward offset (meters)
         * VisionConstants.kSidewaysToCamera, // Side offset (meters)
         * VisionConstants.kGroundToCamera, // Height offset (meters)
         * 0.0, // Roll (degrees)
         * 0.0, // Pitch (degrees)
         * VisionConstants.kCameraRotation // Yaw (degrees)
         * );
         */

        LimelightHelpers.SetIMUMode(camera, 1);
        // LimelightHelpers.setStreamMode_Standard(camera);
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

    public Pose3d getCameraOffsetToRobot() {
        return LimelightHelpers.getCameraPose3d_RobotSpace(camera);
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

    public void updateAprilTagInformation() {
        if (cameraSeesAprilTag()) {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(camera);
            if (fiducials != null)
                for (RawFiducial fidicual : fiducials) {
                    detectedID = fidicual.id;
                    distanceToAprilTag = fidicual.distToRobot;
                    poseAmbiguity = fidicual.ambiguity;
                }
        }
    }

    public int getAprilTagID() {
        return detectedID;
    }

    public double getAprilTagDistanceToRobot() {
        return distanceToAprilTag;
    }

    public void setLimelightIMU(int mode) {
        LimelightHelpers.SetIMUMode(camera, mode);
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

    public boolean tagIsSigma(LimelightHelpers.PoseEstimate limelightMeasurement) {
        double rotationSpeed = Math
                .abs(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
        // double xSpeed = Math
        //         .abs(RobotContainer.m_swerveSubsystem.getSwerveDrive().getRobotVelocity().vxMetersPerSecond);
        // double ySpeed = Math
        //         .abs(RobotContainer.m_swerveSubsystem.getSwerveDrive().getRobotVelocity().vyMetersPerSecond);

        double estimatedDistance = limelightMeasurement.avgTagDist;

        int tagsDetected = limelightMeasurement.tagCount;

        if (rotationSpeed >= Math.PI * 2)
            return false;
        else if (tagsDetected <= 0) 
            return false;
        else 
            return true;
    }

    public boolean cameraSeesCorrectTag() {

        if (!cameraSeesAprilTag())
            return false;

        Pose2d aprilTagPose = aprilTagFieldLayout.getTagPose(getAprilTagID()).get().toPose2d();
        double aprilTagRotation;
        double multiplier = Math
                .round(aprilTagPose.getRotation().getRadians() / Math.abs(aprilTagPose.getRotation().getRadians()));

        if (aprilTagPose.getRotation().getRadians() == 0) {
            aprilTagRotation = Math.PI;
        }

        else {
            aprilTagRotation = aprilTagPose.getRotation().getRadians() - (Math.PI * multiplier);
        }

        double rotationDifference = Math
                .abs(m_swerveDrive.getPose().getRotation().getRadians() - aprilTagRotation);
        return rotationDifference <= Math.toRadians(25);
    }

    public void updateOdometryWithMegaTag1() {
        if (cameraSeesAprilTag()) {
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);

            double rotationSpeed = Math
                    .abs(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
            boolean shouldRejectUpdate = rotationSpeed < 6.28319 && getAprilTagArea() > 50; // 360 degrees

            if (tagIsSigma(limelightMeasurement)) {
                m_swerveDrive.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds,
                        VecBuilder.fill(.5, .5, 9999999));
            }

        }
    }

    public void updateOdometryWithMegaTag2() {
        if (cameraSeesAprilTag()) {
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(camera);

            if (limelightMeasurement != null) {

                double rotationSpeed = Math.abs(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
                boolean shouldRejectUpdate = rotationSpeed < 6.28319 && getAprilTagArea() > 50; // 360 degrees

                m_swerveDrive.setVisionMeasurementStdDevs(
                        VecBuilder.fill(VisionConstants.kStdDevs, VisionConstants.kStdDevs, 9999999));

                Pose2d cameraPose = limelightMeasurement.pose;

                if (tagIsSigma(limelightMeasurement)) {
                    m_swerveDrive.addVisionMeasurement(
                            limelightMeasurement.pose,
                            limelightMeasurement.timestampSeconds);
                    // RobotContainer.m_swerveSubsystem.getSwerveDrive().updateOdometry();
                }
            }
        }

    }

    @Override
    public void periodic() {

        VisionConstants.kPY2 = SmartDashboard.getNumber("kPY Close", VisionConstants.kPY2);
        VisionConstants.kIY2 = SmartDashboard.getNumber("kIY Close", VisionConstants.kIY2);
        VisionConstants.kDY2 = SmartDashboard.getNumber("kDY Close", VisionConstants.kDY2);

        VisionConstants.kPY = SmartDashboard.getNumber("kPY Far", VisionConstants.kPY);
        VisionConstants.kIY = SmartDashboard.getNumber("kIY Far", VisionConstants.kIY);
        VisionConstants.kDY = SmartDashboard.getNumber("kDY Far", VisionConstants.kDY);

        VisionConstants.leftReefToAprilTagOffset = SmartDashboard.getNumber("Left Reef offset",
                VisionConstants.leftReefToAprilTagOffset);
        VisionConstants.rightReefToAprilTagOffset = SmartDashboard.getNumber("Right Reef offset",
                VisionConstants.rightReefToAprilTagOffset);

        VisionConstants.yControllerTolerance = SmartDashboard.getNumber("Y Controller Tolerance",
                VisionConstants.yControllerTolerance);

        VisionConstants.kLocalizationDisLim = SmartDashboard.getNumber("April tag limit",
                VisionConstants.kLocalizationDisLim);

        VisionConstants.kStdDevs = SmartDashboard.getNumber("STD DEVS", VisionConstants.kStdDevs);

        updateAprilTagInformation();
        SmartDashboard.putNumber("Dist2Tag", getAprilTagDistanceToRobot());

        // double robotYaw =
        // RobotContainer.m_swerveSubsystem.getSwerveDrive().getPose().getRotation().getDegrees();
        m_swerveDrive.updateOdometry();
        LimelightHelpers.SetRobotOrientation(VisionConstants.kCameraName,
            m_swerveDrive.getPose().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
        updateOdometryWithMegaTag2();
    }
}