package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

/**
 * Manually increases the target degrees
 */
public class ClawForward extends Command{
    ClawSubsystem subsystem;

    public ClawForward(ClawSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        this.subsystem.forward();
    }
}
