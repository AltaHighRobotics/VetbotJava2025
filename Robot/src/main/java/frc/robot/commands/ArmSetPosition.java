package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmSetPosition extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;

    private double elevatorTargetPercentage;
    private double clawTargetDegrees;

    private String name;

    public ArmSetPosition(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, double elevatorTargetPercentage, double clawTargetDegrees, String name) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.name = name;


        if (elevatorTargetPercentage < 0 || elevatorTargetPercentage > 1) {
            throw new Error("Elevator Target Position must be between 0 and 1");
        }

        if (clawTargetDegrees < 0 || clawTargetDegrees > 360) {
            throw new Error("Claw Target Degrees must be between 0 and 360");
        }

        this.clawTargetDegrees = clawTargetDegrees;
        this.elevatorTargetPercentage = elevatorTargetPercentage;
        
        String fullNameId = String.format("%s state", name);
        // SmartDashboard.putNumber(fullNameId + " Elevator", this.elevatorTargetPercentage);
        // SmartDashboard.putNumber(fullNameId + " Claw", this.clawTargetDegrees);
    }

    @Override
    public void execute() {
        String fullNameId = String.format("%s state", name);
        // this.elevatorTargetPercentage = SmartDashboard.getNumber(fullNameId + " Elevator", this.elevatorTargetPercentage);
        // this.clawTargetDegrees = SmartDashboard.getNumber(fullNameId + " Claw", this.clawTargetDegrees);

        this.clawSubsystem.setRev(clawTargetDegrees/ 360); // May not be working
        this.elevatorSubsystem.setHeight(elevatorTargetPercentage);
    }
}
