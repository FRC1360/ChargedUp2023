package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.Constants;
import frc.robot.commands.ShoulderCommand;

public class ShoulderSubsystem extends SubsystemBase {

    private static CANSparkMax masterShoulder;
    private static CANSparkMax slaveShoulder;
    private static RelativeEncoder masterShoulderEncoder;
    private static RelativeEncoder slaveShoulderEncoder;


    public ShoulderSubsystem (int MasterShoulderID, int SlaveShoulderID) {
        masterShoulder = new CANSparkMax(MasterShoulderID, MotorType.kBrushless);
        slaveShoulder = new CANSparkMax(SlaveShoulderID, MotorType.kBrushless);

        masterShoulderEncoder = masterShoulder.getEncoder();
        slaveShoulderEncoder = slaveShoulder.getEncoder();

        masterShoulder.setIdleMode(IdleMode.kBrake); 
        slaveShoulder.setIdleMode(IdleMode.kBrake); 

        slaveShoulder.follow(masterShoulder); // FIXME should the motors be inverted?
    } 

    public void setZero() {
        masterShoulderEncoder.setPosition(0.0);
        slaveShoulderEncoder.setPosition(0.0);
    }

    public void setSpeed(double speed) {
        masterShoulder.set(speed);
        slaveShoulder.set(speed);
    }

    public double getDegrees() {
        return masterShoulderEncoder.getPosition() / Constants.TICKS_PER_ANGLE_PIVOT;
    }

    public int getStepsfromDegrees(int degrees) {
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

    public double getPositionOfEncoder() {
        return masterShoulderEncoder.getPosition();
    }
}
