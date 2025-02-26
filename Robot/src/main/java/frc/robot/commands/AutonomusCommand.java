package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomusCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDriveSubsystem driveSubsystem;
    private double lifetimeSeconds;
    private boolean isDone;
    private double time;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonomusCommand(SwerveDriveSubsystem driveSubsystem, double lifetimeSeconds) {
      this.driveSubsystem = driveSubsystem;
      this.lifetimeSeconds = lifetimeSeconds;
      this.isDone = false;
      addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      this.driveSubsystem.FOReset();
      this.time = System.currentTimeMillis() / 1000.0;
      this.isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      final double nowInSeconds = System.currentTimeMillis() / 1000.0;

      if (nowInSeconds - this.time <= this.lifetimeSeconds) {
        final double forwardSpeed = 0;
        final double strafeSpeed = -0.5;
        final double rotationSpeed = 0;
        final double speedScaling = 1;

        this.driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed, speedScaling, true);
      } 
      
      else if (nowInSeconds - this.time <= this.lifetimeSeconds + 2) {
        this.stopDrive();
      } 
      
      else {
        this.isDone = true;
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      this.stopDrive();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return this.isDone;
    }

    public void stopDrive() {
      this.driveSubsystem.drive(0, 0, 0, 0, true);
    }
}