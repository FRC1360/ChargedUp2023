package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax; //I don't think this is right
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class ClawSubsystem extends SubsystemBase {
    
    private CANSparkMax motor; // the motor to open and close the claw
    private int deviceId = 60; // 5 is placeholder number. 
    private RelativeEncoder encoder;
    
    

    public ClawSubsystem() { 
        this.motor = new CANSparkMax(deviceId, MotorType.kBrushless);
    }

    public boolean shouldMotorStopMoving(int whenItShouldClose) { //I don't know how many counts is would take to close so placeholder. 
        encoder = motor.getEncoder();
        return encoder.getPosition() <= whenItShouldClose; //Basically it returns if motor passes certain # of revolution
    }

    public void stop(){
        motor.stopMotor();
    }

    public void set(double speed){
        motor.set(speed);
    }

}
