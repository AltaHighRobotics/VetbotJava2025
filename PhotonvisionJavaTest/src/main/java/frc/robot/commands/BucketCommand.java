package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class BucketCommand extends Command {
    enum BucketType {
      EXTEND,
      RETRACT
    }

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BucketSubsystem bucketSubsystem;
    private BucketType bucketType;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public BucketCommand(BucketSubsystem bucketSubsystem, BucketType bucketType) {
      this.bucketSubsystem = bucketSubsystem;
      this.bucketType = bucketType;
      addRequirements(bucketSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      switch (this.bucketType) {
        case EXTEND:
          this.bucketSubsystem.setSpeed(SpeedConstants.BUCKET_SPEED);
          break;

        case RETRACT:
          this.bucketSubsystem.setSpeed(-SpeedConstants.BUCKET_SPEED);
          break;
      }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      this.bucketSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}