package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Controls the elevator
 * This system works the same as the claw
 * We can't just make the motor go one way or the other way because gravity will cause it to fall
 * So we use PID to keep it going to / at a set target position
 * Our manual controls simply change that target position
 * This also makes the state system real easy
 * 
 * Our calcutations are in percentages (0 - 1)
 */
public class ElevatorSubsystem extends SubsystemBase{
    private SparkFlex motorController1;
    private SparkFlex motorController2;

    private PIDController pidController;

    private double targetHeightPercentage = 0.0;

    public ElevatorSubsystem() {
        super();
        this.motorController1 = new SparkFlex(ElevatorConstants.TURN_1_ID, MotorType.kBrushless);
        this.motorController2 = new SparkFlex(ElevatorConstants.TURN_2_ID, MotorType.kBrushless);

        final double P = ElevatorConstants.P;
        final double I = ElevatorConstants.I;
        final double D = ElevatorConstants.D;
        this.pidController = new PIDController(P, I, D);

        // Used for shuffleboard, so we can tweak PID at runtime
        SmartDashboard.putNumber("Elevator P", P);
        SmartDashboard.putNumber("Elevator I", I);
        SmartDashboard.putNumber("Elevator D", D);
    }

    /**
     * Sets the target height for the elevator
     * Will get clamped between 0 and 1
     * @param height A percentage (0 to 1)
     */
    public void setHeight(final double heightPercentage) {
        targetHeightPercentage = MathUtil.clamp(heightPercentage, 0, 1);
    }

    /**
     * This calculates PID and tells the motors the correct volts to get there
     * This is meant to be called in a default command (Looping in the background all the time)
     */
    public void moveToTargetHeight() { // Meant to be called each tick
        targetHeightPercentage = MathUtil.clamp(targetHeightPercentage, 0, 1);
        
        final double targetPositionRevolutions = targetHeightPercentage * ElevatorConstants.TOP_MAG;

        // Grabs the updated values from shuffleboard
        final double newP = SmartDashboard.getEntry("Elevator P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Elevator I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Elevator D").getDouble(0);
        this.pidController.setPID(newP, newI, newD);


        // We are going to track the position of each motor seperatly
        final double currentPositionRevolutions1 = this.getHeight(this.motorController1.getEncoder());
        final double currentPositionRevolutions2 = this.getHeight(this.motorController2.getEncoder());
        double motorOutput1 = this.pidController.calculate(currentPositionRevolutions1, targetPositionRevolutions);
        double motorOutput2 = this.pidController.calculate(currentPositionRevolutions2, -targetPositionRevolutions);

        // Clamp limits the motor speed, should probably use the max output speed but oh well this works too
        motorOutput1 = MathUtil.clamp(motorOutput1, -ElevatorConstants.MOTOR_SPEED, ElevatorConstants.MOTOR_SPEED);
        motorOutput2 = MathUtil.clamp(motorOutput2, -ElevatorConstants.MOTOR_SPEED, ElevatorConstants.MOTOR_SPEED);

        // Make it move half as fast when going down (Gravity makes it go down quicker)
        // if (motorOutput < 0) {
        //     motorOutput *= 0.5;
        // }

        motorController1.setVoltage(motorOutput1);
        motorController2.setVoltage(motorOutput2);
    }

    /**
     * Gets the current height of the elevator
     * @param encoder Should be from motor1 or motor2
     * @return Height as revolutions
     */
    public double getHeight(RelativeEncoder encoder) {
        final double realMotorAngle = encoder.getPosition();
        return realMotorAngle;
    }

    /**
     * Gets the current height of the elevator
     * @return Height as percentage (0.0 to 1.0)
     */
    public double getHeightAsPercentage(RelativeEncoder encoder) {
        return getHeight(encoder) / ElevatorConstants.TOP_MAG;
    }

    public void goUp() {
        targetHeightPercentage += ElevatorConstants.MANUAL_SPEED;
    }

    public void goDown() {
        targetHeightPercentage -= ElevatorConstants.MANUAL_SPEED;
    }

    public void stop() {
        targetHeightPercentage = getHeightAsPercentage(this.motorController1.getEncoder());
        targetHeightPercentage = getHeightAsPercentage(this.motorController2.getEncoder());
        motorController1.set(0);
        motorController2.set(0);
    }
}

