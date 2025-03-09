package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmSetPosition extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;

    private double elevatorTargetPercentage;
    private double clawTargetDegrees;

    public ArmSetPosition(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, double elevatorTargetPercentage, double clawTargetDegrees) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;

        if (elevatorTargetPercentage < 0 || elevatorTargetPercentage > 1) {
            throw new Error("Elevator Target Position must be between 0 and 1");
        }

        if (clawTargetDegrees < 0 || clawTargetDegrees > 360) {
            throw new Error("Claw Target Degrees must be between 0 and 360");
        }

        this.clawTargetDegrees = clawTargetDegrees;
        this.elevatorTargetPercentage = elevatorTargetPercentage;
    }

    @Override
    public void execute() {
        this.clawSubsystem.setRev(clawTargetDegrees/ 360); // May not be working
        this.elevatorSubsystem.setHeight(elevatorTargetPercentage);
    }
}
