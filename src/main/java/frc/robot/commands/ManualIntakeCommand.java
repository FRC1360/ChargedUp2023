package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;

public class ManualIntakeCommand extends CommandBase{
    private IntakeSubsystem intake; // that piston that open and closes the claw
    //make a command that will take in a value that will spin it in a certain speed
    private DoubleSupplier userInputSpeed; // the speed given by the controller

    public ManualIntakeCommand(IntakeSubsystem intake, DoubleSupplier userInputSpeed) {
        this.intake = intake;
        this.userInputSpeed = userInputSpeed;
        addRequirements(intake);
        //withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    
    public boolean isTeleopOver(){
        return false; // placeholder for whether telop is over
    }
    @Override
    public void execute() {
        intake.intake(userInputSpeed.getAsDouble());        
    }

    @Override
    public boolean isFinished() { 
        return isTeleopOver();
    }
}