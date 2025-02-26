// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.kauailabs.navx.frc;

import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.SwerveDriveConstants;


public class SwerveDriveSubsystem extends SubsystemBase {

  private Translation2d frontLeftLocation;
  private Translation2d frontRightLocation;
  private Translation2d backLeftLocation;
  private Translation2d backRightLocation;

  private SwerveModuleSubsystem frontLeftModule;
  private SwerveModuleSubsystem frontRightModule;
  private SwerveModuleSubsystem backLeftModule;
  private SwerveModuleSubsystem backRightModule;

  private boolean turnLock;
  private AHRS gyro;

  private double maxOut;
  
  public SwerveDriveSubsystem() {
    final double P = PIDConstants.P;
    final double I = PIDConstants.I;
    final double D = PIDConstants.D;
    final double ctc2 = SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER / 2.0;

    this.frontLeftLocation = new Translation2d(ctc2, ctc2);
    this.frontRightLocation = new Translation2d(ctc2, -ctc2);
    this.backLeftLocation = new Translation2d(-ctc2, ctc2);
    this.backRightLocation = new Translation2d(-ctc2, -ctc2);

    this.frontLeftModule = new SwerveModuleSubsystem(
      MotorIDConstants.FRONT_LEFT_DRIVE_ID, 
      MotorIDConstants.FRONT_LEFT_TURN_ID, 
      P, I, D
    );

    this.frontRightModule = new SwerveModuleSubsystem(
      MotorIDConstants.FRONT_RIGHT_DRIVE_ID, 
      MotorIDConstants.FRONT_RIGHT_TURN_ID, 
      P, I, D
    );

    this.backLeftModule = new SwerveModuleSubsystem(
      MotorIDConstants.BACK_LEFT_DRIVE_ID, 
      MotorIDConstants.BACK_LEFT_TURN_ID, 
      P, I, D
    );

    this.backRightModule = new SwerveModuleSubsystem(
      MotorIDConstants.BACK_RIGHT_DRIVE_ID, 
      MotorIDConstants.BACK_RIGHT_TURN_ID, 
      P, I, D
    );

    this.turnLock = false;
    this.gyro = new AHRS(NavXComType.kMXP_SPI);

    this.setMaxOutput(SpeedConstants.SWERVE_MAX_OUTPUT);
  }

  public void setMaxOutput(double maxOutput) {
    this.maxOut = maxOutput;
    this.frontLeftModule.setMaxOut(maxOutput);
    this.frontRightModule.setMaxOut(maxOutput);
    this.backLeftModule.setMaxOut(maxOutput);
    this.backRightModule.setMaxOut(maxOutput);
  }

  public void FOReset() {
    this.gyro.zeroYaw();
  }

  public void lockTurn() {
    this.turnLock = true;
  }

  public void unlockTurn() {
    this.turnLock = false;
  }
}
