package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoPutDownCommand extends CommandBase{
    private IntakeSubsystem intake; // that piston that open and closes the claw
    private double speed; 

    public AutoPutDownCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {      
        intake.putDown(speed);
    }

    @Override
    public void end(boolean interruptible) { 
    }

    @Override
    public boolean isFinished() { 
        intake.stop();
        return true;
    }

}
