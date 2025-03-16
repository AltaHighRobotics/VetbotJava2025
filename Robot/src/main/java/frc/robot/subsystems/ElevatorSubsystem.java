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
    private static SparkFlex motorController1 = new SparkFlex(ElevatorConstants.TURN_1_ID, MotorType.kBrushless);
    private static SparkFlex motorController2 = new SparkFlex(ElevatorConstants.TURN_2_ID, MotorType.kBrushless);

    private static PIDController pidController = new PIDController(
        ElevatorConstants.P,
        ElevatorConstants.I,
        ElevatorConstants.D
    );

    private static double targetHeightPercentage = 0.0;

    private ElevatorSubsystem() {
        // SmartDashboard.putNumber("Elevator P", P);
        // SmartDashboard.putNumber("Elevator I", I);
        // SmartDashboard.putNumber("Elevator D", D);
    }

    /**
     * @param height A percentage (0 to 1)
     */
    public static void setHeight(final double heightPercentage) {
        targetHeightPercentage = MathUtil.clamp(heightPercentage, 0.01, 1);
    }

    public static void moveToTargetHeight() { // Meant to be called each tick
        targetHeightPercentage = MathUtil.clamp(targetHeightPercentage, 0, 1);
        SmartDashboard.putNumber("Elevator Current Height", ElevatorSubsystem.getHeightAsPercentage(motorController1.getEncoder()));
        
        final double targetPositionRevolutions = targetHeightPercentage * ElevatorConstants.TOP_MAG;
        final double currentPositionRevolutions1 = ElevatorSubsystem.getHeight(ElevatorSubsystem.motorController1.getEncoder());
        final double currentPositionRevolutions2 = ElevatorSubsystem.getHeight(ElevatorSubsystem.motorController2.getEncoder());

        final double newP = SmartDashboard.getEntry("Elevator P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Elevator I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Elevator D").getDouble(0);
        ElevatorSubsystem.pidController.setPID(newP, newI, newD);

        double motorOutput1 = ElevatorSubsystem.pidController.calculate(currentPositionRevolutions1, targetPositionRevolutions);
        double motorOutput2 = ElevatorSubsystem.pidController.calculate(currentPositionRevolutions2, -targetPositionRevolutions);
        // Clamp limits the motor speed, should probably use the max output speed but oh well ElevatorSubsystem.works too
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
    public static double getHeight(RelativeEncoder encoder) {
        final double realMotorAngle = encoder.getPosition();
        return realMotorAngle;
    }

    /**
     * Gets the current height of the elevator
     * @return Height as percentage (0.0 to 1.0)
     */
    public static double getHeightAsPercentage(RelativeEncoder encoder) {
        return getHeight(encoder) / ElevatorConstants.TOP_MAG;
    }

    public static void goUp() {
        targetHeightPercentage += ElevatorConstants.MANUAL_SPEED;
    }

    public static void goDown() {
        targetHeightPercentage -= ElevatorConstants.MANUAL_SPEED;
    }

    public static void stop() {
        targetHeightPercentage = getHeightAsPercentage(ElevatorSubsystem.motorController1.getEncoder());
        targetHeightPercentage = getHeightAsPercentage(ElevatorSubsystem.motorController2.getEncoder());
        motorController1.set(0);
        motorController2.set(0);
    }
}

