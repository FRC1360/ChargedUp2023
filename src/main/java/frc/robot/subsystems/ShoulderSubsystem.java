package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {

    private static CANSparkMax shoulder;
    private static RelativeEncoder shoulderEncoder;


    public ShoulderSubsystem (int shoulderID) {
        shoulder = new CANSparkMax(shoulderID, MotorType.kBrushless);

        shoulderEncoder = shoulder.getEncoder();

        shoulder.setIdleMode(IdleMode.kBrake); 
    } 

    public static CANSparkMax getShoulderMotor() {
        return shoulder;
    } 

    public static RelativeEncoder getShoulderEncoder() {
        return shoulderEncoder;
    }

    public void setZero() {
        getShoulderEncoder().setPosition(0.0);
    }

    public static void setSpeed(double speed) {
        shoulder.set(speed);
    }
}
