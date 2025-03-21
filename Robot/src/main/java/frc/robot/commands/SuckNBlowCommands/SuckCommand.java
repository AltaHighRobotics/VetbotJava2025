package frc.robot.commands.SuckNBlowCommands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SuckNBlowConstants;
import frc.robot.subsystems.SuckNBlowSubsystem;
import frc.robot.subsystems.SuckNBlowSubsystem.OralType;

/** An example command that uses an example subsystem. */
public class SuckCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SuckNBlowSubsystem subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SuckCommand(SuckNBlowSubsystem subsystem) {
      this.subsystem = subsystem;
      addRequirements(subsystem);
    }

    /**
     * We need to use full power to get the ball in but not to keep it
     * So after a few seconds we lower the motor power
     */
    @Override
    public void initialize() {
      switch (this.subsystem.oralType) {
        case SUCK:
          this.subsystem.set(OralType.STOP);
          break;
        default:
          this.subsystem.set(OralType.SUCK);
      }
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}