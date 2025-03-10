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

    private RelativeEncoder encoder;
    private PIDController pidController;

    private double targetHeightPercentage = 0.0;

    public ElevatorSubsystem() {
        super();
        this.motorController1 = new SparkFlex(ElevatorConstants.TURN_1_ID, MotorType.kBrushless);
        this.motorController2 = new SparkFlex(ElevatorConstants.TURN_2_ID, MotorType.kBrushless);


        this.encoder = motorController1.getEncoder();

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
        if (heightPercentage < 0 || heightPercentage > 1 ) {
            throw new Error("Height should be between 0 and 1");
        } 

        targetHeightPercentage = heightPercentage;
    }

    public void moveToTargetHeight() { // Meant to be called each tick
        final double targetPositionRevolutions = targetHeightPercentage * ElevatorConstants.TOP_MAG;
        final double currentPositionRevolutions = this.getHeight();

        final double newP = SmartDashboard.getEntry("Elevator P").getDouble(0);
        final double newI = SmartDashboard.getEntry("Elevator I").getDouble(0);
        final double newD = SmartDashboard.getEntry("Elevator D").getDouble(0);
        this.pidController.setPID(newP, newI, newD);

        double motorOutput = this.pidController.calculate(currentPositionRevolutions, targetPositionRevolutions);
        System.out.printf("Elevator Motor raw: %.6f\n", motorOutput);

        // Clamp limits the motor speed, should probably use the max output speed but oh well this works too
        motorOutput = MathUtil.clamp(motorOutput, -ElevatorConstants.MOTOR_SPEED, ElevatorConstants.MOTOR_SPEED);

        // Make it move half as fast when going down (Gravity makes it go down quicker)
        // if (motorOutput < 0) {
        //     motorOutput *= 0.5;
        // }

        motorController1.setVoltage(motorOutput);
        motorController2.setVoltage(-motorOutput);
    }

    /**
     * Gets the current height of the elevator
     * @return Height as revolutions
     */
    public double getHeight() {
        final double realMotorAngle = this.encoder.getPosition();
        return realMotorAngle;
    }

    /**
     * Gets the current height of the elevator
     * @return Height as percentage (0.0 to 1.0)
     */
    public double getHeightAsPercentage() {
        return getHeight() / ElevatorConstants.TOP_MAG;
    }

    public void goUp() {
        targetHeightPercentage += 0.01;
    }

    public void goDown() {
        targetHeightPercentage -= 0.01;
    }

    public void stop() {
        targetHeightPercentage = getHeightAsPercentage();
        motorController1.set(0);
        motorController2.set(0);
    }
}

