package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax; //I don't think this is right
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw extends SubsystemBase {
    
    private CANSparkMax motor; // the motor to open and close the claw
    private int deviceId = 5; //I'm not too sure how CANsparkMax works so this part may need to be heavily modified :(
    private RelativeEncoder encoder;
    private double speed = 0.5;
    private boolean status = true; //true = open false = closed

    public Claw() { 
        this.motor = new CANSparkMax(deviceId, MotorType.kBrushless); // I don't know what I'm supposed to put here for "type" so let this be the placeholder
    }

    public boolean shouldMotorStopMoving(int whenItShouldClose) { //I don't know how many counts is would take to close so placeholder. 
        encoder = motor.getEncoder();
        return encoder.getPosition() <= whenItShouldClose; //Basically it returns if motor passes certain # of revolution
    }

    public void stop(){
        motor.stopMotor();
    }
    
    

    public void open(){
        motor.set(speed); 
        if (shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stopMotor();
        }
    }

    public void close(){
        motor.set(-speed); //not sure if num should be - or + be it's opposite of close
        if (shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stopMotor();
        }
    }

    public void toggle(){ //If claw open, it will close. If claw closed, it will open.
        if (status) {
            close();
        }
        else{
            open();
        }
    }


}
