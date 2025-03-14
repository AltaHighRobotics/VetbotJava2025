package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class ToggleRobotOriented extends Command {
    private SwerveDriveSubsystem drive;    

    public ToggleRobotOriented(SwerveDriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        this.drive.FIELD_ORIENTED = !this.drive.FIELD_ORIENTED;
    }
}
