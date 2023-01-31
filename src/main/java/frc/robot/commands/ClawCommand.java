package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase{
    private ClawSubsystem motor; // that piston that open and closes the claw
    private boolean status = true; //true = open false = closed
    private double speed = 0.5; //Also placeholder! IDK how fast the motor should be spinning.

    public ClawCommand(ClawSubsystem motor) {
        this.motor = motor;
        addRequirements(motor);
    }

    public void open(){
        motor.set(speed); 
        if (motor.shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stop();
        }
    }

    public void close(){
        motor.set(-speed); //not sure if num should be - or + be it's opposite of close
        if (motor.shouldMotorStopMoving(5)) { //the five is a placeholder
            motor.stop();
        }
    }

    public void toggle(){ //If claw open, it will close. If claw closed, it will open.
        if (status) {
            close();
        }
        else{
            open();
        }
    }

    @Override
    public void execute() {
        toggle();       
    }
}