package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

/** An example command that uses an example subsystem. */
public class SwitchToRobotOriented extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDriveSubsystem driveSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwitchToRobotOriented(SwerveDriveSubsystem driveSubsystem) {
      this.driveSubsystem = driveSubsystem;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.driveSubsystem.FIELD_ORIENTED = false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // // Returns true when the command should end.
    // @Override
    // public boolean isFinished() {
    //   this.driveSubsystem.FIELD_ORIENTED = false;
    //   return false;
    // }
}