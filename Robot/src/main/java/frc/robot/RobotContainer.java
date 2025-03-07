// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.Swerve.ResetOrientationCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorMoveToTarget;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick driverController;

  private SwerveDriveSubsystem drive;
  private ElevatorSubsystem elevatorSubsystem;

  public RobotContainer() {
    this.driverController = new Joystick(InputConstants.DRIVER_CONTROLLER_PORT);

    this.drive = new SwerveDriveSubsystem();
    this.elevatorSubsystem  = new ElevatorSubsystem();

    configureBindings();

    this.elevatorSubsystem.setDefaultCommand(new ElevatorMoveToTarget(this.elevatorSubsystem));
    this.drive.setDefaultCommand(new SwerveDriveCommand(drive, driverController));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    JoystickButton gyroResetButton = new JoystickButton(driverController, 11);
    gyroResetButton.onTrue(new ResetOrientationCommand(this.drive));
    
    JoystickButton elevatorUpButton = new JoystickButton(driverController, 3);
    JoystickButton elevatorDownButton = new JoystickButton(driverController, 2);
    elevatorUpButton.whileTrue(new ElevatorUp(this.elevatorSubsystem));
    elevatorDownButton.whileTrue(new ElevatorDown(this.elevatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.stationAlign(drive, this.apriltagSubsystem);
    return null;
  }
}
