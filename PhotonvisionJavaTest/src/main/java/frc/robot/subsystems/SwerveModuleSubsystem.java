// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import pabeles.concurrency.IntOperatorTask.Max;

import com.revrobotics.spark.SparkLowLevel;

import static edu.wpi.first.units.Units.Rotation;

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

  public Rotation2d getEncoder() {
    final double tau = Math.PI * 2;
    final double gearRatio = SwerveDriveConstants.SWERVE_TURN_GEAR_RATIO;
    return new Rotation2d(this.turnEncoder.getPosition() * tau * gearRatio);
  }

  public void setMaxOut(double value) {
    this.maxOut = value;
  }

  public void setDesiredState(final SwerveModuleState desiredState) {
    final double tau = Math.PI * 2;
    final double gearRatio = SwerveDriveConstants.SWERVE_TURN_GEAR_RATIO;

    final Rotation2d encoderRotation = this.getEncoder();

    SwerveModuleState state = desiredState;

    state.angle = desiredState.angle.times(-1);
    state.optimize(encoderRotation);
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    final double driveOuput = state.speedMetersPerSecond;
    
    final double turnOutput = this.turningPIDController.calculate(
      this.turnEncoder.getPosition() * tau * gearRatio, state.angle.getRadians()
    );

    this.drive.set(Math.max(-this.maxOut, Math.min(driveOuput, this.maxOut)));
    this.turn.setVoltage(turnOutput);
  }
}
