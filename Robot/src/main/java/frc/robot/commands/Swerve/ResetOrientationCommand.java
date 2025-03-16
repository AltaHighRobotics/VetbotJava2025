package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class ResetOrientationCommand extends Command {
    @Override
    public void execute() {
      SwerveDriveSubsystem.resetOrientation();
    }
}