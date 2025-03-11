package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

/**
 * Meant to be a default command, makes the motor use pid to move towards it target
 */
public class ClawGoToTarget extends Command{
    ClawSubsystem subsystem;

    public ClawGoToTarget(ClawSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        this.subsystem.moveToTarget();
    }
}