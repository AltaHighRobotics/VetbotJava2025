package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Meant to be a default command, makes the motor use pid to move towards it target
 */
public class ElevatorMoveToTarget extends Command {
    ElevatorSubsystem subsystem;
    
    public ElevatorMoveToTarget(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        this.subsystem.moveToTargetHeight();
    }
}
