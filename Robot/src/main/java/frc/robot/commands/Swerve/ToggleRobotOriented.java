package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class ToggleRobotOriented extends Command {
    @Override
    public void initialize() {
        SwerveDriveSubsystem.FIELD_ORIENTED = !SwerveDriveSubsystem.FIELD_ORIENTED;
    }
}
