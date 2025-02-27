package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TravelToApriltagCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private SwerveDriveSubsystem drive;
    private ApriltagSubsystem apriltagSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TravelToApriltagCommand(SwerveDriveSubsystem drive, ApriltagSubsystem apriltagSubsystem) {
      this.drive = drive;
      this.apriltagSubsystem = apriltagSubsystem;
      addRequirements(drive, apriltagSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        final PhotonTrackedTarget target = this.apriltagSubsystem.getFirstTarget();
        final Transform3d pose = target.getBestCameraToTarget();
        final double xMetersAway = pose.getX();
        final double yMetersAway = pose.getY();
        final Rotation3d rotationAway = pose.getRotation();

        final double driveXSpeed = Math.abs(xMetersAway) / xMetersAway; // Make sure they are in terms of -1 to 1
        final double driveYSpeed = Math.abs(yMetersAway) / yMetersAway;
        final double driveRotation = rotationAway.getAngle();
        final double driveSpeed = 0.7;

        drive.drive(driveYSpeed, driveXSpeed, driveRotation, driveSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { return true; }
}
