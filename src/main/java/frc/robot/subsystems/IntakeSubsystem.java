package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        this.leadMotor = new CANSparkMax(Constants.LEAD_INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.followingMotor = new CANSparkMax(Constants.FOLLOW_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followingMotor.follow(leadMotor);
    }

    public boolean shouldMotorStopMoving(int whenItShouldClose) { //I don't know how many counts is would take to close so placeholder. 
        encoder = leadMotor.getEncoder();
        return encoder.getPosition() <= whenItShouldClose; //Basically it returns if motor passes certain # of revolution
    }

    public void stop(){
        //leadMotor.stopMotor();
        SmartDashboard.putNumber("Intake motor speed", 0.0); 
    }

    public void activate(double speed){
        //leadMotor.set(speed);
    }

    public void intake(double speed){
        //leadMotor.set(speed); 
        /* 
        if (shouldMotorStopMoving(whenItShouldClose)) { //the five is a placeholder
            stop();
        }
        */
        SmartDashboard.putNumber("Intake motor speed", speed); 
    }

}
