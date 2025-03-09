package frc.robot.commands.SuckNBlowCommands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SuckNBlowConstants;
import frc.robot.subsystems.SuckNBlowSubsystem;
import frc.robot.subsystems.SuckNBlowSubsystem.OralType;

/** An example command that uses an example subsystem. */
public class BlowCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SuckNBlowSubsystem subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public BlowCommand(SuckNBlowSubsystem subsystem) {
      this.subsystem = subsystem;
      addRequirements(subsystem);
    }

    /**
     * Blows for a few seconds to shoot the ball and then stops the motor
     */
    @Override
    public void execute() {
      this.subsystem.set(OralType.BLOW);
    }

    @Override
    public void end(boolean interrupted) {
      this.subsystem.set(OralType.STOP);
    }
}