package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public final class ClawSubsystem extends SubsystemBase{
    private static SparkMax motor = new SparkMax(ClawConstants.TURN_ID, MotorType.kBrushless);
    private static PIDController pidController = new PIDController(
        ClawConstants.P,
        ClawConstants.I,
        ClawConstants.D
    );
    
    private static double targetRev = 0;

    private ClawSubsystem() {
        // SmartDashboard.putNumber("Claw P", P);
        // SmartDashboard.putNumber("Claw I", I);
        // SmartDashboard.putNumber("Claw D", D);
    }

    public static void moveToTarget() {
        ClawSubsystem.targetRev = MathUtil.clamp(targetRev, 0.01, 1);
        SmartDashboard.putNumber("Claw Current Angle", -ClawSubsystem.getRev() * 360);

        // Adjust the PID to shuffleboard
        final double newP = SmartDashboard.getEntry("Claw P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Claw I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Claw D").getDouble(0);
        ClawSubsystem.pidController.setPID(newP, newI, newD);

        final double motorRawOutput = ClawSubsystem.pidController.calculate(ClawSubsystem.getRev(), -ClawSubsystem.targetRev);
        final double limitedMotorOutput = MathUtil.clamp(motorRawOutput, -ClawConstants.MOTOR_MAX_OUTPUT, ClawConstants.MOTOR_MAX_OUTPUT);
        motor.setVoltage(limitedMotorOutput);
    }

    public static void setRev(double targetRev) {
        ClawSubsystem.targetRev = targetRev;
    }

    public static double getRawRev() {
        return ClawSubsystem.motor.getEncoder().getPosition();
    }

    public static double getRev() {
        return getRawRev() / ClawConstants.REV_FOR_FULL_ROTATION;
    }

    public static void forward() {
        setRev(ClawSubsystem.targetRev + 0.005);
    }

    public static void backward() {
        setRev(ClawSubsystem.targetRev - 0.005);
    }

    public static void stop() {
        System.out.println("Claw Rotation Stoped");
        setRev(ClawSubsystem.getRev());
        ClawSubsystem.motor.set(0);
    }
}

