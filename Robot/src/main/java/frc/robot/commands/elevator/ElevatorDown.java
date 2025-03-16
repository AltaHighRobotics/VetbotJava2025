package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDown extends Command {
    @Override
    public void execute() {
        ElevatorSubsystem.goDown();
    }
}
