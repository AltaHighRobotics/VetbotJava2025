package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class AprilTestCommand extends Command {
    PhotonVisionSubsystem subsystem;
    SwerveDriveSubsystem drive;

    public AprilTestCommand(PhotonVisionSubsystem subsystem, SwerveDriveSubsystem drive) {
        this.subsystem = subsystem;
        this.drive = drive;
        addRequirements(subsystem, drive);
    }

    @Override
    public void execute() {
        // this.drive.drive(0, 0, 0, 0, true);
        PhotonTrackedTarget target = this.subsystem.getBestTarget();
        if (target == null) { return; }

        double ySpeed = 0;
        double xSpeed = 0;
        final double rotation = 0;
        final double speed = 0.4;

        System.out.println(target.getArea());
        if (target.getArea() < 5) {
            xSpeed = -1;
            ySpeed = 0;
        }


        this.drive.drive(ySpeed, xSpeed, rotation, speed, true);
    }
}
