package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawGoToTarget extends Command{
    ClawSubsystem subsystem;

    public ClawGoToTarget(ClawSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        this.subsystem.moveToTarget();
    }
}