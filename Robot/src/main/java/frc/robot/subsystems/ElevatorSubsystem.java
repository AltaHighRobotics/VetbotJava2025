package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

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

        SmartDashboard.putNumber("Elevator P", P);
        SmartDashboard.putNumber("Elevator I", I);
        SmartDashboard.putNumber("Elevator D", D);
    }

    /**
     * @param height A percentage (0 to 1)
     */
    public void setHeight(final double heightPercentage) {
        targetHeightPercentage = MathUtil.clamp(heightPercentage, 0.2, 1);
    }

    public void moveToTargetHeight() { // Meant to be called each tick
        targetHeightPercentage = MathUtil.clamp(targetHeightPercentage, 0, 1);
        
        final double targetPositionRevolutions = targetHeightPercentage * ElevatorConstants.TOP_MAG;
        final double currentPositionRevolutions1 = this.getHeight(this.motorController1.getEncoder());
        final double currentPositionRevolutions2 = this.getHeight(this.motorController2.getEncoder());

        final double newP = SmartDashboard.getEntry("Elevator P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Elevator I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Elevator D").getDouble(0);
        this.pidController.setPID(newP, newI, newD);

        double motorOutput1 = this.pidController.calculate(currentPositionRevolutions1, targetPositionRevolutions);
        double motorOutput2 = this.pidController.calculate(currentPositionRevolutions2, -targetPositionRevolutions);
        System.out.printf("Elevator Motor raw: %.6f\n", motorOutput1);

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

