package frc.robot.commands.shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderHoldCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;
    private DoubleSupplier intakeSpeed;

    public ShoulderHoldCommand(ShoulderSubsystem shoulder, DoubleSupplier intakeSpeed) {
        this.shoulder = shoulder;
        this.intakeSpeed = intakeSpeed;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.holdPIDController.reset();
    }

    @Override
    public void execute() {
        double target = this.shoulder.getTargetAngle();
        double input = this.shoulder.getShoulderAngle();
        double speed = this.shoulder.holdPIDController.calculate(target, input);

        double kF = this.shoulder.shoulderFeedForward.calculate(target, 
                                                                this.shoulder.getAngluarVelocity()); 
        
        speed += kF; 
        if (this.intakeSpeed.getAsDouble() > 0.1) speed = -0.1; 
        

        this.shoulder.setShoulderNormalizedVoltage(speed);
        //this.shoulder.setShoulderNormalizedVoltage(0.1); 

        SmartDashboard.putNumber("Shoulder_Hold_Output", speed + kF);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
