package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShoulderCommand;

public class ShoulderSubsystem extends SubsystemBase {

    private static CANSparkMax shoulder;
    private static RelativeEncoder shoulderEncoder;


    public ShoulderSubsystem (int shoulderID) {
        shoulder = new CANSparkMax(shoulderID, MotorType.kBrushless);

        shoulderEncoder = shoulder.getEncoder();

        shoulder.setIdleMode(IdleMode.kBrake); 
    } 

    public void setZero() {
        shoulderEncoder.setPosition(0.0);
    }

    public static void setSpeed(double speed) {
        shoulder.set(speed);
    }

    public double getDegrees() {
        return shoulderEncoder.getPosition() / Constants.TICKS_PER_ANGLE_PIVOT;
    }

    public static int getStepsfromDegrees(int degrees) {
        return Constants.TICKS_PER_ANGLE_PIVOT * degrees;
    }

    public void setTargetLow() {
        ShoulderCommand.setAngle(5); // TODO Enter low target angle
    }
    
    public void setTargetMiddle() {
        ShoulderCommand.setAngle(20); // TODO Enter Middle target angle
    }

    public void setTargetHigh() {
        ShoulderCommand.setAngle(45); // TODO Enter High target angle
    }

    public static double getPositionOfEncoder() {
        return shoulderEncoder.getPosition();
    }
}
