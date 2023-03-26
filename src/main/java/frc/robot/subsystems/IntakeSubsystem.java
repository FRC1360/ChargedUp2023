package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax leadMotor; // the motor to open and close the claw
    private CANSparkMax followingMotor; // the motor that follows the lead motor

    public IntakeSubsystem() { 
        this.leadMotor = new CANSparkMax(Constants.LEAD_INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.followingMotor = new CANSparkMax(Constants.FOLLOW_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followingMotor.follow(leadMotor);
    }

    public void stop(){
        leadMotor.stopMotor();
        SmartDashboard.putNumber("Intake motor speed", 0.0); 
    }

    public void intake(double speed){
        leadMotor.set(speed);
        SmartDashboard.putNumber("Intake motor speed", speed); 
    }

}
