package frc.robot.visions;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class CameraInterface extends SubsystemBase {
    private final NetworkTable camera;

    private final LinearFilter xTranslationFilterController;
    private double xTranslationFiltered = 0;
    private int aprilTagID = -1;
    private double aprilTagArea = 0;
    private final String LIMELIGHT_KEY;
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    

     /**
     * Creates a new camera object for each camera attached to the Raspberry Pi.
     * Be sure the name assigned to the camera on PhotonVision is the same as the one that is used here.
     * 
     * @param cameraName
     *      */     
    public CameraInterface(String cameraName) {
        camera = NetworkTableInstance.getDefault().getTable("limelight");
        xTranslationFilterController = LinearFilter.movingAverage(10);
        LIMELIGHT_KEY = cameraName;
          
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
     * @return Whether the April Tag detected by the camera is an April Tag on the reef or not.
     */ 
    public boolean isReefAprilTag() {
        switch (getTargetAprilTagID()) {
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
        return camera.getEntry("tv").getDouble(0) != 0 && isReefAprilTag();
    }

    public double getAprilTagArea() {
        return camera.getEntry("ta").getDouble(0);
    }

    public double getTx() {
        // tx: Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
        // degrees | LL2: -29.8 to 29.8 degrees)
        return LimelightHelpers.getTX(LIMELIGHT_KEY);
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


    public int getTargetAprilTagID() {
        return (int) camera.getEntry("tid").getInteger(-1);
    }


    @Override
    public void periodic() {

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
}