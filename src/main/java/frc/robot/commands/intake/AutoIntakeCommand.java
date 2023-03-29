package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeCommand extends CommandBase{
    private IntakeSubsystem intake;
    private double speed; 

    public AutoIntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void execute() {      
        intake.intake(speed);
    }

    @Override
    public void end(boolean interruptible) { 
        intake.stop();
    }

    @Override
    public boolean isFinished() { 
        return this.intake.intakeSensor.get(); 
    }

}