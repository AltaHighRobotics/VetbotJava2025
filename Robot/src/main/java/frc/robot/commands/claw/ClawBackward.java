package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawBackward extends Command{
    @Override
    public void execute() {
        ClawSubsystem.backward();
    }
}
