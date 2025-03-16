package frc.robot.commands.elevator;

import java.util.EventListener;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUp extends Command {
    @Override
    public void execute() {
        ElevatorSubsystem.goUp();
    }
}
