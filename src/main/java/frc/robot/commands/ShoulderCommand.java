package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends CommandBase {
    private static double degrees = 0.0;
    private static ShoulderSubsystem subsystem;

    public ShoulderCommand(ShoulderSubsystem ssystem, double degrees) {
        subsystem = ssystem;
        this.degrees = degrees; 
        addRequirements(ssystem);
    }

    @Override
    public void initialize() { 
        //subsystem.setZero();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("command running", true);
        subsystem.setAngle(degrees);
        SmartDashboard.putNumber("current angle", subsystem.getAngle());
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAngle()-degrees <= 0.5 && subsystem.getAngle()-degrees >= -0.5) return true;
        else {
            
            return false; 
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("command running", false);
        subsystem.setSpeed(0);
    }
}
