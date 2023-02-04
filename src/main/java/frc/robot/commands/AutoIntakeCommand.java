package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeCommand extends CommandBase{
    private IntakeSubsystem intake; // that piston that open and closes the claw
    private double speed = 0.5; //Also placeholder! IDK how fast the motor should be spinning.
    private int whenItShouldClose = 5;

    public AutoIntakeCommand(IntakeSubsystem motor) {
        this.intake = intake;
        addRequirements(motor);
        withInterruptBehavior(InterruptionBehavior.kCancelSelf); //makes it so telop takes priority to auto command.
    }

    public void intake(){
        intake.activate(speed); 
        if (intake.shouldMotorStopMoving(whenItShouldClose)) { //the five is a placeholder
            intake.stop();
        }
    }

    public void putDown(){
        intake.activate(-speed); //not sure if num should be - or + be it's opposite of close
        if (intake.shouldMotorStopMoving(whenItShouldClose)) { //the five is a placeholder
            intake.stop();
        }
    }

    public boolean isRobotReadyToDropOff(){
        return true; // Placeholder for the sensor that will tell the robot wether it is ready to putdown object
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
        return isRobotReadyToDropOff();
    }

}