package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristHoldCommand  extends CommandBase{

    private WristSubsystem wrist;

    private double ff = 0.0; 

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
        
        if (!this.wrist.getAngularVelocity().isNaN()) { 
            ff = this.wrist.wristFeedForward.calculate(Math.toRadians(this.wrist.getWristOffset()), 
                                                            this.wrist.getAngularVelocity()); 
        } 

        SmartDashboard.putNumber("Wrist_Hold_FF", ff); 
        //double speed = pidOutput + ff; 

        //if (Math.abs(speed) > 0.50) speed =  Math.copySign(0.5, speed); 
        this.wrist.setWristNormalizedVoltage(ff);         
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
