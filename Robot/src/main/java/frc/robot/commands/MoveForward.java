package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** An example command that uses an example subsystem. */
public class MoveForward extends Command {
    private final SwerveDriveSubsystem drive;


    public MoveForward(SwerveDriveSubsystem drive) {
      this.drive = drive;
      addRequirements(drive);
    }

    @Override
    public void execute() {
      final double driveXSpeed = 1.0;
        final double driveYSpeed = 0.0;
        final double driveRotation = 0.0;
        final double driveSpeed = 0.7;

        drive.drive(driveYSpeed, driveXSpeed, driveRotation, driveSpeed);
    }

}