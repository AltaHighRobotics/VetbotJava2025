package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class SwerveDriveCommand extends Command {
  @Override
  public void execute() {
    final double forwardSpeed = -RobotContainer.driverController.getX();
    final double strafeSpeed = this.driverController.getY();
    final double rotationSpeed = this.driverController.getZ();
    final double speedScaling = MathUtil.clamp((-this.driverController.getRawAxis(3) + 1.0) / 2.0, 0.3,1);
  
    this.driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed, speedScaling);
  }
}