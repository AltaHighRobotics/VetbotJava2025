package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** An example command that uses an example subsystem. */
public class HalfSpeedCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDriveSubsystem drive;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public HalfSpeedCommand(SwerveDriveSubsystem drive) {
      this.drive = drive;
      addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      this.drive.setMaxOutput(0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      this.drive.setMaxOutput(SpeedConstants.SWERVE_MAX_OUTPUT);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}