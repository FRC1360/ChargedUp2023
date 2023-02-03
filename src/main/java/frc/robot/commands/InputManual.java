package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

public class InputManual extends CommandBase{
    private Intake intake; // that piston that open and closes the claw
    //make a command that will take in a value that will spin it in a certain speed
    public InputManual(Intake motor) {
        this.intake = intake;
        addRequirements(intake);
    }
    private double speed = 0.5;
    
    @Override
    public void execute() {
        intake.set();
        
    }

}