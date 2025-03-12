package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawSetDegrees extends Command {
    ClawSubsystem subsystem;    
    double targetDegrees;

    public ClawSetDegrees(ClawSubsystem subsystem, double targetDegrees) {
        this.subsystem = subsystem;

        if (targetDegrees < 0 || targetDegrees > 1) {
            throw new Error("Target Height for ElevatorSetHeight must be between 0 and 1");
        }

        this.targetDegrees = targetDegrees;
    }

    @Override
    public void execute() {
        this.subsystem.setRev(this.targetDegrees / 360); // This logic is not finished and might not work
    }
}
