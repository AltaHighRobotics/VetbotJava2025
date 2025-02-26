package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
    enum IntakeType {
      INTAKE,
      OUTTAKE,
    }

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private IntakeType intakeType;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeType intakeType) {
      this.intakeSubsystem = intakeSubsystem;
      this.intakeType = intakeType;
      addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      switch (this.intakeType) {
        case INTAKE:
          this.intakeSubsystem.setSpeed(SpeedConstants.INTAKE_SPEED);
          break;

        case OUTTAKE:
          this.intakeSubsystem.setSpeed(-SpeedConstants.OUTTAKE_SPEED);
          break;
      }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      this.intakeSubsystem.setSpeed(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}