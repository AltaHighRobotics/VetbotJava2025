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
    @Override
    public void initialize() {
      switch (SuckNBlowSubsystem.oralType) {
        case SUCK:
          SuckNBlowSubsystem.set(OralType.STOP);
          break;
        default:
          SuckNBlowSubsystem.set(OralType.SUCK);
      }
    }
}