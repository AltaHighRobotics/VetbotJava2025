// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

import com.revrobotics.spark.SparkLowLevel;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;


public class SwerveModuleSubsystem extends SubsystemBase {
  
  TalonFX drive;
  SparkMax turn;
  RelativeEncoder turnEncoder;
  ProfiledPIDController turningPIDController;
  double maxOut;

  /** Creates a new SwerveModuleSubsystem. */
  public SwerveModuleSubsystem(int driveID, int steerID, double P, double I, double D) {
    this.drive = new TalonFX(driveID, "talonfx");
    this.drive.setNeutralMode(NeutralModeValue.Brake);

    this.turn = new SparkMax(steerID, SparkLowLevel.MotorType.kBrushless);
    this.turnEncoder = this.turn.getEncoder(); // Zero wheels before power on

    this.turningPIDController = new ProfiledPIDController(
      P, I, D,
      new TrapezoidProfile.Constraints(
        SwerveDriveConstants.MODULE_MAX_ANGULAR_VELOCITY,
        SwerveDriveConstants.MODULE_MAX_ANGULAR_ACCELERATION
      )

    );
    this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    this.maxOut = 0;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
