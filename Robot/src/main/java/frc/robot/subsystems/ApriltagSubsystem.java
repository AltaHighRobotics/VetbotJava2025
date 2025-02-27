// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
