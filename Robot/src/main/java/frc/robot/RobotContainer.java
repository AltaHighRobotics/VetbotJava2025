// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.ArmSetPosition;
import frc.robot.commands.Autos;
import frc.robot.commands.SuckNBlowCommands.BlowCommand;
import frc.robot.commands.SuckNBlowCommands.SuckCommand;
import frc.robot.commands.Swerve.ResetOrientationCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.commands.Swerve.SwitchToFieldOriented;
import frc.robot.commands.Swerve.SwitchToRobotOriented;
import frc.robot.commands.claw.ClawBackward;
import frc.robot.commands.claw.ClawForward;
import frc.robot.commands.claw.ClawGoToTarget;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorMoveToTarget;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SuckNBlowSubsystem;
import frc.robot.subsystems.SuckNBlowSubsystem.OralType;
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
  private final Joystick stateController;

  private SwerveDriveSubsystem drive;
  private ElevatorSubsystem elevatorSubsystem;
  private ClawSubsystem clawSubsystem;
  private SuckNBlowSubsystem suckNBlowSubsystem;

  public RobotContainer() {
    this.driverController = new Joystick(InputConstants.DRIVER_CONTROLLER_PORT);
    this.stateController = new Joystick(1);

    this.drive = new SwerveDriveSubsystem();
    this.elevatorSubsystem  = new ElevatorSubsystem();
    this.clawSubsystem = new ClawSubsystem();
    this.suckNBlowSubsystem = new SuckNBlowSubsystem();

    configureBindings();

    this.clawSubsystem.setDefaultCommand(new ClawGoToTarget(clawSubsystem));
    this.elevatorSubsystem.setDefaultCommand(new ElevatorMoveToTarget(this.elevatorSubsystem));
    this.drive.setDefaultCommand(new SwerveDriveCommand(drive, this.driverController));
  }

  private void addStateBinding(int buttonID, double elevatorHeight, double clawDegrees) {
    JoystickButton button = new JoystickButton(this.stateController, buttonID);
    button.whileTrue(new ArmSetPosition(elevatorSubsystem, clawSubsystem, elevatorHeight, clawDegrees));
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

    JoystickButton gyroResetButton = new JoystickButton(driverController, 7);
    gyroResetButton.onTrue(new ResetOrientationCommand(this.drive));
    
    JoystickButton elevatorUpButton = new JoystickButton(driverController, 5);
    JoystickButton elevatorDownButton = new JoystickButton(driverController, 3);
    elevatorUpButton.whileTrue(new ElevatorUp(this.elevatorSubsystem));
    elevatorDownButton.whileTrue(new ElevatorDown(this.elevatorSubsystem));

    JoystickButton clawForwardButton = new JoystickButton(driverController, 6);
    JoystickButton clawBackwardButton = new JoystickButton(driverController, 4);
    clawForwardButton.whileTrue(new ClawForward(this.clawSubsystem));
    clawBackwardButton.whileTrue(new ClawBackward(this.clawSubsystem));

    JoystickButton suckButton = new JoystickButton(driverController, 1);
    JoystickButton blowButton = new JoystickButton(driverController, 2);
    suckButton.whileTrue(new SuckCommand(this.suckNBlowSubsystem));
    blowButton.whileTrue(new BlowCommand(this.suckNBlowSubsystem));

    addStateBinding(8, 0, 216); // L1
    addStateBinding(9, 0.35, 275); // L2
    addStateBinding(10, 0.57, 275); // L3
    addStateBinding(11, 0.91, 275); // L4
    addStateBinding(7, 0, 153); // CG
    addStateBinding(6, 0.25, 235); // RP1
    addStateBinding(5, 0.5, 235); // RP2
    addStateBinding(4, 0, 235); // BS


    JoystickButton makeFIeldOriented = new JoystickButton(driverController, 12);
    makeFIeldOriented.whileTrue(new SwitchToRobotOriented(drive));
    makeFIeldOriented.whileFalse(new SwitchToFieldOriented(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.stationAlign(drive, this.apriltagSubsystem);
    return Autos.moveForwardStupid(drive);
  }
}
