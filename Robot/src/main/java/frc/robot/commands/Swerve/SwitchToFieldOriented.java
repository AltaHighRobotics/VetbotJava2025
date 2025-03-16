package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class SwitchToFieldOriented extends Command {
    @Override
    public void execute() {
      SwerveDriveSubsystem.FIELD_ORIENTED = true;
    }
}