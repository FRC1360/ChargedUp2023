package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem motor; // that piston that open and closes the claw
    private double speed = 0.5; //Also placeholder! IDK how fast the motor should be spinning.

    public IntakeCommand(IntakeSubsystem motor) {
        this.motor = motor;
        addRequirements(motor);
        getInterruptionBehavior();//makes it so telop takes priority to auto command.
    }

    public void intake(){
        motor.activate(speed); 
        if (motor.shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stop();
        }
    }

    public void putDown(){
        motor.activate(-speed); //not sure if num should be - or + be it's opposite of close
        if (motor.shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stop();
        }
    }

    public boolean isRobotReadyToDropOff(){
        return true;
        // Since the motor insn't going to keep moving during transport, 
        // we should have some way for the robot to determine if it is ready to place down the object other then encoder ticks. 

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
        if (isRobotReadyToDropOff()) {
            return true;
        }
        else{
            return false;
        }
        
    }
}