package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetHeight extends Command {
    double targetHeightPercentage;

    public ElevatorSetHeight(ElevatorSubsystem subsystem, double targetHeightPercentage) {
        if (targetHeightPercentage < 0 || targetHeightPercentage > 1) {
            throw new Error("Target Height for ElevatorSetHeight must be between 0 and 1");
        }

        this.targetHeightPercentage = targetHeightPercentage;
    }

    @Override
    public void execute() {
        ElevatorSubsystem.setHeight(this.targetHeightPercentage);
    }
}
