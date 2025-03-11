package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Manully adjusts the elevator target lower
 */
public class ElevatorDown extends Command {
    ElevatorSubsystem subsystem;
    
    public ElevatorDown(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        this.subsystem.goDown();
    }
}
