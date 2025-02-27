// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConsants;

public class ApriltagSubsystem extends SubsystemBase {

  private PhotonCamera camera;
  private List<PhotonTrackedTarget> targets;

  /** Creates a new ApriltagSubsystem. */
  public ApriltagSubsystem() {
    super(); 

    this.camera = new PhotonCamera(PhotonVisionConsants.CAMERA_NAME);
    this.targets = new ArrayList<PhotonTrackedTarget>();
  }

  public PhotonTrackedTarget getHighestID(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget bestTarget = targets.get(0);

    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() > bestTarget.getFiducialId()) {
        bestTarget = target;
      }
    }

    return bestTarget;
  }

  List<PhotonTrackedTarget> refresh() {
    PhotonPipelineResult result = this.camera.getLatestResult();

    if (result.hasTargets()) {
      this.targets = result.getTargets();
      return this.targets;
    } else {
      return null;
    }
  }

  public PhotonTrackedTarget getFirstTarget() {
    if (targets.size() == 0) { return null; }
    return targets.get(0);
  }

  public double getTargetYaw(int id) {
    List<PhotonTrackedTarget> newTargets = this.targets;
    if (newTargets != null) {
      for (PhotonTrackedTarget target : newTargets) {
        if (target.getFiducialId() == id) {
          return target.getYaw();
        }
      }
    }

    return 0;
  }
}
