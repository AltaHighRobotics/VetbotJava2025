package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax motorController1;
    private SparkMax motorController2;

    private RelativeEncoder encoder;
    private PIDController pidController;

    private double targetHeightPercentage = 0.0;

    public ElevatorSubsystem() {
        super();
        this.motorController1 = new SparkMax(ElevatorConstants.TURN_1_ID, MotorType.kBrushless);
        this.motorController2 = new SparkMax(ElevatorConstants.TURN_2_ID, MotorType.kBrushless);

        final double P = ElevatorConstants.P;
        final double I = ElevatorConstants.I;
        final double D = ElevatorConstants.D;
        this.pidController = new PIDController(P, I, D);
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

        double motorOutput = this.pidController.calculate(currentPositionRevolutions, targetPositionRevolutions);

        // Clamp limits the motor speed, should probably use the max output speed but oh well this works too
        motorOutput = MathUtil.clamp(motorOutput, -ElevatorConstants.MOTOR_SPEED, ElevatorConstants.MOTOR_SPEED);

        // Make it move half as fast when going down (Gravity makes it go down quicker)
        if (motorOutput < 0) {
            motorOutput *= 0.5;
        }

        motorController1.set(motorOutput);
        motorController2.set(-motorOutput);
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

