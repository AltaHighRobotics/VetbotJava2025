// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command stationAlign(SwerveDriveSubsystem drive, ApriltagSubsystem apriltagSubsystem) {
    // return Commands.sequence(new FollowApriltagCommand(drive, apriltagSubsystem));
  // }

  public static Command moveForwardStupid(SwerveDriveSubsystem drive ) {
    return Commands.sequence(new MoveForwardStupid(drive));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
