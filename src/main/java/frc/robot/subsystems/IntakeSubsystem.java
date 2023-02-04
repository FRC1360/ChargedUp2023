package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;



public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax leadMotor; // the motor to open and close the claw
    private CANSparkMax followingMotor; // the motor that follows the lead motor

    private RelativeEncoder encoder;
    private int whenItShouldClose;

    public IntakeSubsystem() { 
        this.leadMotor = new CANSparkMax(Constants.ClawSubsystem.deviceId, MotorType.kBrushless);
        this.followingMotor = new CANSparkMax(61, MotorType.kBrushless);
        followingMotor.follow(leadMotor);
    }

    public boolean shouldMotorStopMoving(int whenItShouldClose) { //I don't know how many counts is would take to close so placeholder. 
        encoder = leadMotor.getEncoder();
        return encoder.getPosition() <= whenItShouldClose; //Basically it returns if motor passes certain # of revolution
    }

    public void stop(){
        leadMotor.stopMotor();
    }

    public void activate(double speed){
        leadMotor.set(speed);
    }

    public void intake(double speed){
        leadMotor.set(speed); 
        if (shouldMotorStopMoving(whenItShouldClose)) { //the five is a placeholder
            stop();
        }
    }

    public void putDown(double speed){
        leadMotor.set(-speed); //not sure if num should be - or + be it's opposite of close
        if (shouldMotorStopMoving(whenItShouldClose)) { //the five is a placeholder
            
            stop();
        }
    }

}
