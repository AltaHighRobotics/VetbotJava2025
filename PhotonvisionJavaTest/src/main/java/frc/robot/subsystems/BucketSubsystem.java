// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class BucketSubsystem extends SubsystemBase {

  private VictorSPX motor;

  public BucketSubsystem() {
    super();

    this.motor = new VictorSPX(MotorIDConstants.BUCKET_ID);
    this.motor.setInverted(true);
  }

  public void setSpeed(double speed) {
    this.motor.set(ControlMode.PercentOutput, speed);
  }
}
