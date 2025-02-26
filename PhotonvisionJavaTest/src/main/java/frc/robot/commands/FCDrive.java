package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** An example command that uses an example subsystem. */
public class FCDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDriveSubsystem driveSubsystem;
    private CommandJoystick driverController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FCDrive(SwerveDriveSubsystem driveSubsystem, CommandJoystick driverController) {
      this.driveSubsystem = driveSubsystem;
      this.driverController = driverController;
      addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      final double forwardSpeed = this.driverController.getX();
      final double strafeSpeed = this.driverController.getY();
      final double rotationSpeed = this.driverController.getZ();
      final double speedScaling = (-(this.driverController.getRawAxis(3)) + 1.0) / 2.0;
    
      this.driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed, speedScaling, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}