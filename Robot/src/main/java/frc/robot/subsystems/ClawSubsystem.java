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

public class ClawSubsystem extends SubsystemBase{
    private SparkMax motor;
    private PIDController pidController;
    
    private double targetRev = 0;

    public ClawSubsystem() {
        super();

        this.motor = new SparkMax(ClawConstants.TURN_ID, MotorType.kBrushless);

        final double P = ClawConstants.P;
        final double I = ClawConstants.I;
        final double D = ClawConstants.D;
        this.pidController = new PIDController(P, I, D);

        SmartDashboard.putNumber("Claw P", P);
        SmartDashboard.putNumber("Claw I", I);
        SmartDashboard.putNumber("Claw D", D);
        // this.motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void moveToTarget() {
        System.out.printf("Target Rev: %.6f\n", this.targetRev);
        System.out.printf("Current Rev: %.6f\n", this.getRev());

        this.targetRev = MathUtil.clamp(targetRev, 0.15, 1);

        // Adjust the PID to shuffleboard
        final double newP = SmartDashboard.getEntry("Claw P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Claw I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Claw D").getDouble(0);
        this.pidController.setPID(newP, newI, newD);

        final double motorRawOutput = this.pidController.calculate(this.getRev(), -this.targetRev);
        final double limitedMotorOutput = MathUtil.clamp(motorRawOutput, -ClawConstants.MOTOR_MAX_OUTPUT, ClawConstants.MOTOR_MAX_OUTPUT);
        motor.setVoltage(limitedMotorOutput);
    }

    public void setRev(double targetRev) {
        this.targetRev = targetRev;
    }

    public double getRawRev() {
        return this.motor.getEncoder().getPosition();
    }

    public double getRev() {
        return getRawRev() / ClawConstants.REV_FOR_FULL_ROTATION;
    }

    public void forward() {
        setRev(this.targetRev + 0.005);
    }

    public void backward() {
        setRev(this.targetRev - 0.005);
    }

    public void stop() {
        System.out.println("Claw Rotation Stoped");
        setRev(this.getRev());
        this.motor.set(0);
    }
}

