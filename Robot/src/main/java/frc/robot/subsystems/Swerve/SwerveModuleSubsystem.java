// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

import com.revrobotics.spark.SparkLowLevel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;


public class SwerveModuleSubsystem extends SubsystemBase {
  
  private TalonFX drive;
  private SparkMax turn;
  private RelativeEncoder turnEncoder;
  private ProfiledPIDController turningPIDController;
  private double maxOut;

  /**
  * Construct module, pid, and start encoder
  *
  * @param driveID ID of the TalonFX, should be 2-5
  * @param steerID ID of the SparkMax (turn), should be 22-55
  */
  public SwerveModuleSubsystem(int driveID, int steerID, double P, double I, double D) {
    super();

    this.drive = new TalonFX(driveID, "rio");
    this.drive.setNeutralMode(NeutralModeValue.Coast); // Stop wheel from moving when weren't not driving

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

    // Default maxOutput to 0
    this.maxOut = 0;
  }

  /**
  * Uses the encoder to calculate the current direction of the module
  *
  * @return The Rotation2d of the module
  */
  public Rotation2d getEncoder() {
    final double tau = Math.PI * 2;
    final double gearRatio = SwerveDriveConstants.SWERVE_TURN_GEAR_RATIO;
    return new Rotation2d(this.turnEncoder.getPosition() * tau * gearRatio);
  }

  /**
  * Used to limit the speed of the robot
  *
  * @param value 0 - 1, max speed of the motor, 0.5 would be half speed
  */
  public void setMaxOut(double value) {
    this.maxOut = value;
  }

  /**
  * Uses a SwerveModuleState outputed by kinematics.
  * This of course sets the state, so it will keep running after the function is called once.
  *
  * @param desiredState The desired state of the module from kinematics
  */
  public void setDesiredState(final SwerveModuleState desiredState) {
    final double tau = Math.PI * 2;
    final double gearRatio = SwerveDriveConstants.SWERVE_TURN_GEAR_RATIO;

    // Gets the current angle of the module
    final Rotation2d encoderRotation = this.getEncoder();

    SwerveModuleState state = desiredState;

    state.angle = desiredState.angle.times(-1); // Inverts the angle
    state.optimize(encoderRotation); // Optimizes out unnessary rotatation when there's a faster way
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    final double driveOuput = state.speedMetersPerSecond;
    final double currentRadians = this.turnEncoder.getPosition() * tau * gearRatio;
    
    // Uses PID to tell the SparkMax's how much to rotate
    final double turnOutput = this.turningPIDController.calculate(
      currentRadians, state.angle.getRadians()
    );

    // Actually sets the speed of the motors and how much they need to rotate
    this.drive.set(Math.max(-this.maxOut, Math.min(driveOuput, this.maxOut)));
    this.turn.setVoltage(turnOutput);
  }
}
