package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClawSubsystem extends SubsystemBase{
    private SparkMax motor;
    private PIDController pidController;
    
    private double targetDegrees = -1;

    public ClawSubsystem() {
        super();

        this.motor = new SparkMax(ClawConstants.TURN_ID, MotorType.kBrushless);

        final double P = ClawConstants.P;
        final double I = ClawConstants.I;
        final double D = ClawConstants.D;
        this.pidController = new PIDController(P, I, D);

        // this.motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void moveToTarget() {
        while (this.targetDegrees > 359) {
            this.targetDegrees -= 359;
        }

        final double currentDegrees = this.getDegrees();

        final double targetRad = Math.toRadians(this.targetDegrees);
        final double currentRad = Math.toRadians(currentDegrees);

        // System.out.printf("Claw Current Position: %.5f\n", currentDegrees);
        // System.out.printf("Claw Target Position: %.5f\n", this.targetDegrees);

        final double motorRawOutput = this.pidController.calculate(currentRad, targetRad);
        final double limitedMotorOutput = MathUtil.clamp(motorRawOutput, -ClawConstants.MOTOR_MAX_OUTPUT, ClawConstants.MOTOR_MAX_OUTPUT);
        motor.setVoltage(limitedMotorOutput);
    }

    public void setDegrees(double targetDegrees) {
        this.targetDegrees = targetDegrees;
    }

    public double getDegrees() {
        final double currentMotorRevolutions = this.motor.getEncoder().getPosition();
        double currentMotorPercentage = currentMotorRevolutions / ClawConstants.MOTOR_REVOLUTIONS_FOR_FULL_ROTATION;

        // Make sure motor resets after 359 deg
        while (currentMotorPercentage > 0) {
            currentMotorPercentage --;
        }

        return currentMotorPercentage * 359;
    }

    public void forward() {
        setDegrees(this.targetDegrees + 1);
    }

    public void backward() {
        setDegrees(this.targetDegrees - 1);
    }

    public void stop() {
        System.out.println("Claw Rotation Stoped");
        setDegrees(this.getDegrees());
        this.motor.set(0);
    }
}

