package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax motorController;
    private PIDController pidController;
    private RelativeEncoder encoder;

    public ElevatorSubsystem() {
        super();

        this.motorController = new SparkMax(ElevatorConstants.TURN_ID, MotorType.kBrushless);

        this.encoder = this.motorController.getEncoder();

        final double P = ElevatorConstants.P;
        final double I = ElevatorConstants.I;
        final double D = ElevatorConstants.D;
        this.pidController = new PIDController(P, I, D);
    }

    void setHeight(double height) {
        final double motorOutput = this.pidController.calculate(encoder.getPosition(), height);
        motorController.set(motorOutput);
    }

    double getHeight() {
        return encoder.getPosition();
    }
}
