package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.OrbitTimer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeCommand extends CommandBase{
    private IntakeSubsystem intake;
    private double speed; 

    private OrbitTimer timer; 

    public AutoIntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        this.timer = new OrbitTimer(); 

        addRequirements(intake);
    }

    @Override
    public void initialize() { 
        this.timer.start();
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
        return this.timer.getTimeDeltaSec() > 1.0;  
    }

}