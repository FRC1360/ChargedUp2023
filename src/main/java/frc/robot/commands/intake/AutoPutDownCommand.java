package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        intake.intake(-speed);
    }

    @Override
    public void end(boolean interruptible) { 
        intake.stop();
    }

    @Override
    public boolean isFinished() { 
        return false; // needs to be changed a bit...
    }

    

}
