package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristHoldCommand  extends CommandBase{

    private WristSubsystem wrist;

    public WristHoldCommand(WristSubsystem wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void initialize() { 
        this.wrist.holdPIDController.reset(); 
    }
    
    @Override
    public void execute() {
        double target = this.wrist.getTargetAngle();
        double input = this.wrist.getWristAngle();
        double pidOutput = this.wrist.holdPIDController.calculate(target, input);

        double feedforwardOutput = 0.0;
        
        if (!this.wrist.getAngularVelocity().isNaN()) { 
            feedforwardOutput = this.wrist.wristFeedForward.calculate(Math.toRadians(this.wrist.getWristOffset()), 
                                                                        Math.toRadians(this.wrist.getAngularVelocity())); 
        } 

       
        SmartDashboard.putNumber("Wrist_Hold_FF", feedforwardOutput); 
        double speed = pidOutput + feedforwardOutput; 

        SmartDashboard.putNumber("Wrist_Hold_PID_Output", speed); 

        //if (Math.abs(speed) > 0.50) speed =  Math.copySign(0.5, speed); 
        this.wrist.setWristNormalizedVoltage(speed);         
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
