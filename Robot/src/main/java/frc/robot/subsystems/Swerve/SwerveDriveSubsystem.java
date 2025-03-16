// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of SwerveDriveSubsystem.project.

package frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InputConstants;
import frc.robot.Constants.SwerveDriveConstants;


public final class SwerveDriveSubsystem extends SubsystemBase {

  private static Translation2d frontLeftLocation  = new Translation2d( SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0,  SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0);
  private static Translation2d frontRightLocation = new Translation2d( SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0, -SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0);
  private static Translation2d backLeftLocation   = new Translation2d(-SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0,  SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0);
  private static Translation2d backRightLocation  = new Translation2d(-SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0, -SwerveDriveConstants.SWERVE_MOD_CENTER_TO_CENTER /2.0);

  private static SwerveModuleSubsystem frontLeftModule = new SwerveModuleSubsystem(
    SwerveDriveConstants.FRONT_LEFT_DRIVE_ID, 
    SwerveDriveConstants.FRONT_LEFT_TURN_ID, 
    SwerveDriveConstants.P, SwerveDriveConstants.I, SwerveDriveConstants.D
  );

  private static SwerveModuleSubsystem frontRightModule = new SwerveModuleSubsystem(
    SwerveDriveConstants.FRONT_RIGHT_DRIVE_ID, 
    SwerveDriveConstants.FRONT_RIGHT_TURN_ID, 
    SwerveDriveConstants.P, SwerveDriveConstants.I, SwerveDriveConstants.D
  );

  private static SwerveModuleSubsystem backLeftModule = new SwerveModuleSubsystem(
    SwerveDriveConstants.BACK_LEFT_DRIVE_ID, 
    SwerveDriveConstants.BACK_LEFT_TURN_ID, 
    SwerveDriveConstants.P, SwerveDriveConstants.I, SwerveDriveConstants.D
  );

  private static SwerveModuleSubsystem backRightModule = new SwerveModuleSubsystem(
    SwerveDriveConstants.BACK_RIGHT_DRIVE_ID, 
    SwerveDriveConstants.BACK_RIGHT_TURN_ID, 
    SwerveDriveConstants.P, SwerveDriveConstants.I, SwerveDriveConstants.D
  );

  private static boolean turnLock = false;
  private static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    SwerveDriveSubsystem.frontLeftLocation,
    SwerveDriveSubsystem.frontRightLocation,
    SwerveDriveSubsystem.backLeftLocation,
    SwerveDriveSubsystem.backRightLocation
  );


  public static boolean FIELD_ORIENTED = true;

  private SwerveDriveSubsystem() {}

  /**
  * All axis are based around the gyro, which can be reset at any time.
  * @param ySpeed Speed to set the motors going along the y axis. -1 to 1
  * @param xSpeed Speed to set the motors going along the x axis. -1 to 1
  * @param rot The angular rate of the robot. Radians I think
  * @param speed Scales the speed, if set to 0 the robot won't move. 0 to 1
  */
  public static void drive(double ySpeed, double xSpeed, double rot, double speed) {
    SwerveDriveSubsystem.setMaxOutput(SwerveDriveConstants.SWERVE_MAX_OUTPUT);
    Shuffleboard.getTab("main").addBoolean("FIELD (FALSE IS ROBOT)", () -> SwerveDriveSubsystem.FIELD_ORIENTED);

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

    if (Math.abs(rot) < InputConstants.TURN_DEADBAND || SwerveDriveSubsystem.turnLock) {
      rot = 0;
      axes0 ++;
    } else {
      rot -= Math.copySign(InputConstants.TURN_DEADBAND, rot);
    }

    // Tells the robot which way it needs to go
    ChassisSpeeds chassisSpeeds; 


    if (FIELD_ORIENTED) {
      // v is for Velocity
      final double vxMetersPerSecond = -xSpeed * speed;
      final double vyMetersPerSecond = ySpeed * speed;
      final double omegaRadiansPerSecond = -rot * speed * Math.PI;
      Rotation2d robotAngle = SwerveDriveSubsystem.gyro.getRotation2d();

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, 
                                                            omegaRadiansPerSecond, robotAngle);
    } else { // Robot oriented
      chassisSpeeds = new ChassisSpeeds(xSpeed/2, -ySpeed/2, -rot/2);
    }

    // Uses kinematics to calculate the module states from where we tell it to go
    SwerveModuleState[] swerveModulesStates = SwerveDriveSubsystem.kinematics.toSwerveModuleStates(chassisSpeeds);

    // Makes the wheel turn into an X pattern for defense
    if (axes0 == 3) {
      // X Pattern
      // swerveModulesStates[0].angle = new Rotation2d(Math.PI / 4);
      // swerveModulesStates[1].angle = new Rotation2d(-Math.PI / 4);
      // swerveModulesStates[2].angle = new Rotation2d(-Math.PI / 4);
      // swerveModulesStates[3].angle = new Rotation2d(Math.PI / 4);

      // O Pattern
      swerveModulesStates[0].angle = new Rotation2d(-Math.PI / 4);
      swerveModulesStates[1].angle = new Rotation2d(Math.PI / 4);
      swerveModulesStates[2].angle = new Rotation2d(Math.PI / 4);
      swerveModulesStates[3].angle = new Rotation2d(-Math.PI / 4);
    }

    // Uses our method to set the rotation and speed of the modules from the calculated values
    SwerveDriveSubsystem.frontLeftModule.setDesiredState(swerveModulesStates[0]);
    SwerveDriveSubsystem.frontRightModule.setDesiredState(swerveModulesStates[1]);
    SwerveDriveSubsystem.backLeftModule.setDesiredState(swerveModulesStates[2]);
    SwerveDriveSubsystem.backRightModule.setDesiredState(swerveModulesStates[3]);
  }

  /**
  * Sets the max output for all of the modules
  * @param maxOutput 0 - 1
  */
  public static void setMaxOutput(double maxOutput) {
    SwerveDriveSubsystem.frontLeftModule.setMaxOut(maxOutput);
    SwerveDriveSubsystem.frontRightModule.setMaxOut(maxOutput);
    SwerveDriveSubsystem.backLeftModule.setMaxOut(maxOutput);
    SwerveDriveSubsystem.backRightModule.setMaxOut(maxOutput);
  }

  /**
  * Resets the gyro, which resets the orientation.
  * The x and y axis will be changed to fit how the robot is turned when called.
  */
  public static void resetOrientation() {
    SwerveDriveSubsystem.gyro.zeroYaw();
  }

  public static void lockTurn() {
    SwerveDriveSubsystem.turnLock = true;
  }

  public static void unlockTurn() {
    SwerveDriveSubsystem.turnLock = false;
  }
}
