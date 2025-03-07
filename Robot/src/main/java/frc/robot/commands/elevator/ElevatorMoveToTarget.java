package frc.robot.commands.elevator;

import java.util.EventListener;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToTarget extends Command {
    ElevatorSubsystem subsystem;
    
    public ElevatorMoveToTarget(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        this.subsystem.moveToTargetHeight();
    }
}
