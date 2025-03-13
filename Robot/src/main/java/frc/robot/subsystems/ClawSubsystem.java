package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private GenericEntry pEntry;
    private GenericEntry iEntry;
    private GenericEntry dEntry;

    public ClawSubsystem() {
        super();

        this.motor = new SparkMax(ClawConstants.TURN_ID, MotorType.kBrushless);

        final double P = ClawConstants.P;
        final double I = ClawConstants.I;
        final double D = ClawConstants.D;
        this.pidController = new PIDController(P, I, D);

        ShuffleboardTab tab = Shuffleboard.getTab("PID");
        pEntry = tab.add("Claw P", P).getEntry();
        iEntry = tab.add("Claw I", I).getEntry();
        dEntry = tab.add("Claw D", D).getEntry();
    }

    public void moveToTarget() {
        this.targetRev = MathUtil.clamp(targetRev, 0.01, 1);
        SmartDashboard.putNumber("Claw Current Angle", -this.getRev() * 360);

        // Adjust the PID to shuffleboard
        final double newP = pEntry.getDouble(0);
        final double newI = iEntry.getDouble(0);
        final double newD = dEntry.getDouble(0);
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

