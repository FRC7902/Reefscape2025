package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    private final PhotonCamera m_alignCam;
    public boolean targetDetected = false;
    public double targetYaw = 0.0;
    public double targetDistance = 0.0;


    public CameraSubsystem() {
       m_alignCam = new PhotonCamera("skibidi");
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("April Tag Detected", targetDetected);
        var results = m_alignCam.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 18) {
                        targetYaw = target.getYaw();
                        targetDistance = PhotonUtils.calculateDistanceToTargetMeters(0.5, 1.435, 3, target.getPitch());
                        targetDetected = true;
                    }
                }
            }
        }
    }


}
