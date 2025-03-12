package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

class MoveForwardStupid extends Command {
    SwerveDriveSubsystem drive;

    public MoveForwardStupid(SwerveDriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void execute() {
        drive.drive(0, -1, 0, 0.5);
    }
}