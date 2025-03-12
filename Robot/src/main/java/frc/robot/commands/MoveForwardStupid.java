package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

class MoveForwardStupid extends Command {
    SwerveDriveSubsystem drive;
    private long startTime;

    public MoveForwardStupid(SwerveDriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        final long currentTime = System.currentTimeMillis();
        final long milisecondsAlive = currentTime - this.startTime; 
        if (milisecondsAlive > AutonomousConstants.milisecondsAlive) {
            drive.drive(0, 0, 0, 0);
            return;
        }

        drive.drive(0, 1, 0, AutonomousConstants.driveSpeed);
    }
}