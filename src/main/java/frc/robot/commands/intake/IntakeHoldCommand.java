package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHoldCommand extends CommandBase {

    private IntakeSubsystem intake; 
    
    public IntakeHoldCommand(IntakeSubsystem intake) { 
        this.intake = intake; 

        addRequirements(intake);
    }

    @Override
    public void execute() { 
        this.intake.intake(0.05);
    }
}
