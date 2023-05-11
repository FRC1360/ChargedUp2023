package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristHoldCommand  extends CommandBase{

    private WristSubsystem wrist;

    //private DoubleSupplier intakeSpeed; 

    public WristHoldCommand(WristSubsystem wrist, DoubleSupplier intakeSpeed) {
        this.wrist = wrist;
        //this.intakeSpeed = intakeSpeed; 

        addRequirements(wrist);
    }

    @Override
    public void initialize() { 
        // no longer need to reset PID
    }
    
    @Override
    public void execute() {
        // FF needs to be done by RoboRIO
        double feedforwardOutput = 0.0;
        if (!this.wrist.getAngularVelocity().isNaN()) { 
            feedforwardOutput = this.wrist.wristFeedForward.calculate(
                Math.toRadians(this.wrist.getWristAngleLocal()),
                Math.toRadians(this.wrist.getAngularVelocity())); 
        }

        this.wrist.updateSmartMotion(wrist.holdSlot, this.wrist.getTargetAngle(), feedforwardOutput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
