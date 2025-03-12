package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetHeight extends Command {
    ElevatorSubsystem subsystem;    
    double targetHeightPercentage;

    public ElevatorSetHeight(ElevatorSubsystem subsystem, double targetHeightPercentage) {
        this.subsystem = subsystem;

        if (targetHeightPercentage < 0 || targetHeightPercentage > 1) {
            throw new Error("Target Height for ElevatorSetHeight must be between 0 and 1");
        }

        this.targetHeightPercentage = targetHeightPercentage;
    }

    @Override
    public void execute() {
        this.subsystem.setHeight(this.targetHeightPercentage);
    }
}
