package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuckNBlowConstants;

public final class SuckNBlowSubsystem extends SubsystemBase{
    public enum OralType {
        SUCK,
        BLOW,
        STOP 
    }

    private static SparkMax motor = new SparkMax(SuckNBlowConstants.SPARK_MAX_ID, MotorType.kBrushless);
    public static OralType oralType = OralType.STOP;

    private SuckNBlowSubsystem() {}

    /**
     * Sets the state of the motors
     * @param oralType The direction the motors will go, positive, negitive, or stopped
     */
    public static void set(OralType oralType) {
        SuckNBlowSubsystem.oralType = oralType;
        switch (oralType) {
            case SUCK:
                SuckNBlowSubsystem.motor.set(-SuckNBlowConstants.MOTOR_SUCK_SPEED);
                break;
            case BLOW:
                SuckNBlowSubsystem.motor.set(SuckNBlowConstants.MOTOR_BLOW_SPEED);
                break;
            case STOP:
                SuckNBlowSubsystem.motor.set(0);
                break;
        }
    }
}
