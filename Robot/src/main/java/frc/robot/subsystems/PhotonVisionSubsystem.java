package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    
    public PhotonVisionSubsystem() {
        camera = new PhotonCamera("camera1");
    }

    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) { return null; }
        return result.getBestTarget();
    }
}
