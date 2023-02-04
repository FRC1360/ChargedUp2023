package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualPutdownCommand extends CommandBase {
    private IntakeSubsystem intake; // that piston that open and closes the claw
    //make a command that will take in a value that will spin it in a certain speed
    private double userInputSpeed;

    public ManualPutdownCommand(IntakeSubsystem intake, double userInputSpeed) {
        this.intake = intake;
        this.userInputSpeed = userInputSpeed;
        addRequirements(intake);
        withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean isTeleopOver(){
        return false; // placeholder for whether telop is over
    }
    @Override
    public void execute() {
        intake.putDown(userInputSpeed);        
    }

    @Override
    public boolean isFinished() { 
        return isTeleopOver();
    }
}
