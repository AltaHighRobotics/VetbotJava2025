// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InputConstants;
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

  private SwerveDriveKinematics kinematics;
  
  /**
  * Constructs the drive, creates modules for each of the four motors.
  * To create kinematics we need to give it locations which are created from the disance between the motors.
  * Also Initalized the gyro.
  */
  public SwerveDriveSubsystem() {
    super();

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

    this.kinematics = new SwerveDriveKinematics(
      this.frontLeftLocation,
      this.frontRightLocation,
      this.backLeftLocation,
      this.backRightLocation
    );

    this.gyro.reset();

    this.setMaxOutput(SpeedConstants.SWERVE_MAX_OUTPUT);
  }

  /**
  * All axis are based around the gyro, which can be reset at any time.
  * @param ySpeed Speed to set the motors going along the y axis.
  * @param xSpeed Speed to set the motors going along the x axis.
  * @param rot The angular rate of the robot.
  * @param speed Scales the speed, if set to 0 the robot won't move
  */
  public void drive(double ySpeed, double xSpeed, double rot, double speed) {
    int axes0 = 0;

    if (Math.abs(xSpeed) < InputConstants.DEADBAND) {
      xSpeed = 0;
      axes0 ++;
    }

    if (Math.abs(ySpeed) < InputConstants.DEADBAND) {
      ySpeed = 0;
      axes0 ++;
    }

    if (Math.abs(rot) < InputConstants.TURN_DEADBAND || this.turnLock) {
      rot = 0;
      axes0 ++;
    } else {
      rot -= Math.copySign(InputConstants.TURN_DEADBAND, rot);
    }

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      -xSpeed * speed,
      -ySpeed * speed,
      -rot * speed * Math.PI,
      this.gyro.getRotation2d() 
    );

    if (!InputConstants.FIELD_ORIENTED) {
      chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -rot);
    }

    SwerveModuleState[] swerveModulesStates = this.kinematics.toSwerveModuleStates(chassisSpeeds);

    if (axes0 == 3) {
      swerveModulesStates[0].angle = new Rotation2d(Math.PI / 4);
      swerveModulesStates[1].angle = new Rotation2d(-Math.PI / 4);
      swerveModulesStates[2].angle = new Rotation2d(-Math.PI / 4);
      swerveModulesStates[3].angle = new Rotation2d(Math.PI / 4);
    }

    this.frontLeftModule.setDesiredState(swerveModulesStates[0]);
    this.frontRightModule.setDesiredState(swerveModulesStates[1]);
    this.backLeftModule.setDesiredState(swerveModulesStates[2]);
    this.backRightModule.setDesiredState(swerveModulesStates[3]);
  }

  public void setMaxOutput(double maxOutput) {
    this.frontLeftModule.setMaxOut(maxOutput);
    this.frontRightModule.setMaxOut(maxOutput);
    this.backLeftModule.setMaxOut(maxOutput);
    this.backRightModule.setMaxOut(maxOutput);
  }

  /**
  * Resets the gyro, which resets the orientation.
  * The x and y axis will be changed to fit how the robot is turned when called.
  */
  public void resetOrientation() {
    this.gyro.zeroYaw();
  }

  public void lockTurn() {
    this.turnLock = true;
  }

  public void unlockTurn() {
    this.turnLock = false;
  }
}
