// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InputConstants;
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

    final double P = SwerveDriveConstants.P;
    final double I = SwerveDriveConstants.I;
    final double D = SwerveDriveConstants.D;
    final double ctc2 = SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER / 2.0; // Half the width of the side of the robot

    // Uses the distance between the motors to get Translation2d posisions from the center of the robot
    this.frontLeftLocation = new Translation2d(ctc2, ctc2);
    this.frontRightLocation = new Translation2d(ctc2, -ctc2);
    this.backLeftLocation = new Translation2d(-ctc2, ctc2);
    this.backRightLocation = new Translation2d(-ctc2, -ctc2);

    // Create our modules
    this.frontLeftModule = new SwerveModuleSubsystem(
      SwerveDriveConstants.FRONT_LEFT_DRIVE_ID, 
      SwerveDriveConstants.FRONT_LEFT_TURN_ID, 
      P, I, D
    );

    this.frontRightModule = new SwerveModuleSubsystem(
      SwerveDriveConstants.FRONT_RIGHT_DRIVE_ID, 
      SwerveDriveConstants.FRONT_RIGHT_TURN_ID, 
      P, I, D
    );

    this.backLeftModule = new SwerveModuleSubsystem(
      SwerveDriveConstants.BACK_LEFT_DRIVE_ID, 
      SwerveDriveConstants.BACK_LEFT_TURN_ID, 
      P, I, D
    );

    this.backRightModule = new SwerveModuleSubsystem(
      SwerveDriveConstants.BACK_RIGHT_DRIVE_ID, 
      SwerveDriveConstants.BACK_RIGHT_TURN_ID, 
      P, I, D
    );

    this.turnLock = false;

    this.gyro = new AHRS(NavXComType.kMXP_SPI);

    // Kinematics calculates our movement from x, y, and rot
    this.kinematics = new SwerveDriveKinematics(
      this.frontLeftLocation,
      this.frontRightLocation,
      this.backLeftLocation,
      this.backRightLocation
    );

    this.gyro.reset();

    this.setMaxOutput(SwerveDriveConstants.SWERVE_MAX_OUTPUT);
  }

  /**
  * All axis are based around the gyro, which can be reset at any time.
  * @param ySpeed Speed to set the motors going along the y axis. -1 to 1
  * @param xSpeed Speed to set the motors going along the x axis. -1 to 1
  * @param rot The angular rate of the robot. Radians I think
  * @param speed Scales the speed, if set to 0 the robot won't move. 0 to 1
  */
  public void drive(double ySpeed, double xSpeed, double rot, double speed) {
    int axes0 = 0;

    // Checks if we are actually telling the robot to move or rotate
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

    // Tells the robot which way it needs to go
    ChassisSpeeds chassisSpeeds;
    if (InputConstants.FIELD_ORIENTED) {
      // v is for Velocity
      final double vxMetersPerSecond = -xSpeed * speed;
      final double vyMetersPerSecond = ySpeed * speed;
      final double omegaRadiansPerSecond = -rot * speed * Math.PI;
      Rotation2d robotAngle = this.gyro.getRotation2d();

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, 
                                                            omegaRadiansPerSecond, robotAngle);
    } else { // Not field oriented
      chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -rot);
    }

    // Uses kinematics to calculate the module states from where we tell it to go
    SwerveModuleState[] swerveModulesStates = this.kinematics.toSwerveModuleStates(chassisSpeeds);

    // Makes the wheel turn into an X pattern for defense
    if (axes0 == 3) {
      swerveModulesStates[0].angle = new Rotation2d(Math.PI / 4);
      swerveModulesStates[1].angle = new Rotation2d(-Math.PI / 4);
      swerveModulesStates[2].angle = new Rotation2d(-Math.PI / 4);
      swerveModulesStates[3].angle = new Rotation2d(Math.PI / 4);
    }

    // Uses our method to set the rotation and speed of the modules from the calculated values
    this.frontLeftModule.setDesiredState(swerveModulesStates[0]);
    this.frontRightModule.setDesiredState(swerveModulesStates[1]);
    this.backLeftModule.setDesiredState(swerveModulesStates[2]);
    this.backRightModule.setDesiredState(swerveModulesStates[3]);
  }

  /**
  * Sets the max output for all of the modules
  * @param maxOutput 0 - 1
  */
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
