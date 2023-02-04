package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;



public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax leadMotor; // the motor to open and close the claw
    private CANSparkMax followingMotor; // the motor that follows the lead motor

    private RelativeEncoder encoder;
    

    public IntakeSubsystem() { 
        this.leadMotor = new CANSparkMax(Constants.ClawSubsystem.deviceId, Constants.K_MOTOR_TYPE);
        this.followingMotor = new CANSparkMax(61, Constants.K_MOTOR_TYPE);
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

}
