package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class PhotonSubsystem extends SubsystemBase {
    private PhotonCamera camera;

    public PhotonSubsystem() {
        this.camera = new PhotonCamera("photonvision");
    }    
}
