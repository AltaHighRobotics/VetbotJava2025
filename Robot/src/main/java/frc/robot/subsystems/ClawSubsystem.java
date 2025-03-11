package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Controls the claw, the part that rotates on the arm
 * This system works the same as the elevator
 * We can't just make the motor go one way or the other way because gravity will cause it to fall
 * So we use PID to keep it going to / at a set target position
 * Our manual controls simply change that target position
 * This also makes the state system real easy
 * 
 * Our calculations are in revolutions but we give input with degrees
 * 1 rev = 360 deg
 */
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

        // Smart Dashboard allows us to tweak the values at runtime
        SmartDashboard.putNumber("Claw P", P);
        SmartDashboard.putNumber("Claw I", I);
        SmartDashboard.putNumber("Claw D", D);
    }

    /**
     * This calculates PID and tells the motors the correct volts to get there
     * This is meant to be called in a default command (Looping in the background all the time)
     */
    public void moveToTarget() {
        this.targetRev = MathUtil.clamp(targetRev, 0, 1);

        // Adjust the PID to shuffleboard to tweak at runtime
        final double newP = SmartDashboard.getEntry("Claw P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Claw I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Claw D").getDouble(0);
        this.pidController.setPID(newP, newI, newD);

        final double motorRawOutput = this.pidController.calculate(this.getRev(), this.targetRev);

        // This clamp limit the motor volts to between -Max and Max
        final double limitedMotorOutput = MathUtil.clamp(motorRawOutput, -ClawConstants.MOTOR_MAX_OUTPUT, ClawConstants.MOTOR_MAX_OUTPUT);
        motor.setVoltage(limitedMotorOutput);
    }

    /**
     * Sets the target rev
     * @param targetRev Target in revolutions (0 - 1) use deg / 360 to input degrees
     */
    public void setRev(double targetRev) {
        this.targetRev = targetRev;
    }

    /**
     * Gets the current raw rev (Without our gearRatio/rotations to top)
     * You problably shouldn't use this an should use getRev instead
     * @return The raw rev (Directly graps from the motor encoders position)
     */
    public double getRawRev() {
        return this.motor.getEncoder().getPosition();
    }

    /** 
     * Gets the current rev, with the rotations to top included
     * Will be less that the raw rev
    */
    public double getRev() {
        return getRawRev() / ClawConstants.REV_FOR_FULL_ROTATION;
    }

    /**
     * Used for manul control, adjusts the target
     */
    public void forward() {
        setRev(this.targetRev + 0.03);
    }

    /**
     * Used for manul control, adjusts the target
     */
    public void backward() {
        setRev(this.targetRev - 0.03);
    }

    public void stop() {
        System.out.println("Claw Rotation Stoped");
        setRev(this.getRev());
        this.motor.set(0);
    }
}

