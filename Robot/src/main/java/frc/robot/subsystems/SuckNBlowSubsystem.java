package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuckNBlowConstants;

public class SuckNBlowSubsystem extends SubsystemBase{
    public enum OralType {
        SUCK,
        BLOW,
        STOP 
    }

    private SparkMax motor;

    public SuckNBlowSubsystem() {
        super();
        this.motor = new SparkMax(SuckNBlowConstants.SPARK_MAX_ID, MotorType.kBrushless);
    }

    /**
     * Sets the state of the motors
     * @param oralType The direction the motors will go, positive, negitive, or stopped
     */
    public void set(OralType oralType) {
        switch (oralType) {
            case SUCK:
                this.motor.set(-SuckNBlowConstants.MOTOR_SUCK_SPEED);
                break;
            case BLOW:
                this.motor.set(SuckNBlowConstants.MOTOR_BLOW_SPEED);
                break;
            case STOP:
                this.motor.set(0);
                break;
        }
    }
}
