package frc.robot.commands.shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;

public class ShoulderHoldCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;
    private DoubleSupplier intakeSpeed;
    private ArmShoulderMessenger armMessenger; 

    public ShoulderHoldCommand(ShoulderSubsystem shoulder, ArmShoulderMessenger armMessenger, DoubleSupplier intakeSpeed) {
        this.shoulder = shoulder;
        this.intakeSpeed = intakeSpeed;
        this.armMessenger = armMessenger; 
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
        double speedPidOutput = this.shoulder.holdPIDController.calculate(target, input);

        double kF = this.shoulder.shoulderFeedForward.calculate(target, 
                this.shoulder.getAngluarVelocity()); 
        
        
        double speedOutput = speedPidOutput + kF; 

        double speedOnDistance = speedOutput * Math.pow(1.03, this.armMessenger.getArmDistance());  //base, exponent - this is ton increase the power based on arm distance

        //SmartDashboard.putNumber("Shoulder_Speed_Output_With_Distance", speedOnDistance); 

        double speed = speedOnDistance; 

        if (this.intakeSpeed.getAsDouble() > 0.1 && this.shoulder.getInIntakePosition()) speed = -0.08; // This is to drive the shoulder into the ground while intaking
        

        this.shoulder.setShoulderNormalizedVoltage(speed);
        //this.shoulder.setShoulderNormalizedVoltage(0.1); 

        //SmartDashboard.putNumber("Shoulder_Hold_Output", speed);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
