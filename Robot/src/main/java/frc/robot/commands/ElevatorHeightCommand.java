package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHeightCommand extends Command {
    public enum ElevatorLevel {
        LEVEL1,
        LEVEL2,
        LEVEL3,
    } 
    
    private ElevatorLevel elevatorLevel;
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem, 
                                 ElevatorLevel elevatorLevel) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorLevel = elevatorLevel;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        switch (this.elevatorLevel) {
            case LEVEL1:
                this.elevatorSubsystem.setHeight(ElevatorConstants.LEVEL1_HEIGHT);
                break;
            case LEVEL2:
                this.elevatorSubsystem.setHeight(ElevatorConstants.LEVEL2_HEIGHT);
                break;
            case LEVEL3:
                this.elevatorSubsystem.setHeight(ElevatorConstants.LEVEL3_HEIGHT);
                break;
        }
    }
}
