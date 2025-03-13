package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSubsystem extends SubsystemBase {
    private PhotonCamera camera;

    public PhotonSubsystem() {
        this.camera = new PhotonCamera("photonvision");
    }    

    public PhotonTrackedTarget getTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) { return null; }
        return result.getBestTarget();
    }
}
