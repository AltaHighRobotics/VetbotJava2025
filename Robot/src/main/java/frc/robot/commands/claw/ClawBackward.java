package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

/**
 * Manually decreases the target degrees
 */
public class ClawBackward extends Command{
    ClawSubsystem subsystem;

    public ClawBackward(ClawSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        this.subsystem.backward();
    }
}
