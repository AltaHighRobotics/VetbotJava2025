// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX motorLeft;
  private TalonSRX motorRight;

  public IntakeSubsystem() {
    super();

    this.motorLeft = new TalonSRX(MotorIDConstants.INTAKE_LEFT_ID);

    this.motorRight = new TalonSRX(MotorIDConstants.INTAKE_RIGHT_ID);
    this.motorRight.setInverted(true);
  }

  public void setSpeed(double speed) {
    this.motorLeft.set(TalonSRXControlMode.PercentOutput, speed);
    this.motorRight.set(TalonSRXControlMode.PercentOutput, speed);
  }
}
