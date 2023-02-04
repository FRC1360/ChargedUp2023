package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

public class ManualIntakeCommand extends CommandBase{
    private IntakeSubsystem intake; // that piston that open and closes the claw
    //make a command that will take in a value that will spin it in a certain speed
    private double userInputSpeed; // the speed given by the controller
    public ManualIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
        withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    
    public boolean isTeleopOver(){
        return false; // placeholder for whether telop is over
    }
    @Override
    public void execute() {
        intake.activate(userInputSpeed);        
    }

    @Override
    public boolean isFinished() { 
        if (isTeleopOver()) {
            return true;
        }
        else {
            return false;
        }
    }

}