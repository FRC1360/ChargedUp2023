package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem motor; // that piston that open and closes the claw
    private double speed = 0.5; //Also placeholder! IDK how fast the motor should be spinning.

    public IntakeCommand(IntakeSubsystem motor) {
        this.motor = motor;
        addRequirements(motor);
        getInterruptionBehavior();
    }

    public void intake(){
        motor.set(speed); 
        if (motor.shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stop();
        }
    }

    public void putDown(){
        motor.set(-speed); //not sure if num should be - or + be it's opposite of close
        if (motor.shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stop();
        }
    }

    @Override
    public void initialize(){
        intake();
    }

    @Override
    public void execute() {           
    }

    @Override
    public void end(boolean interruptible) { 
        putDown();
    }

    @Override
    public boolean isFinished() { 
        motor.stop();
        return true;
    }
}