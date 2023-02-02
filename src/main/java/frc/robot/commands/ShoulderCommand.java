package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends CommandBase {
    private static double degrees;
    private static ShoulderSubsystem subsystem;

    public ShoulderCommand(ShoulderSubsystem ssystem, double degrees) {
        subsystem = ssystem;
        addRequirements(ssystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("command running", true);
        subsystem.setAngle(degrees);
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAngle()-degrees <= 0.5 && subsystem.getAngle()-degrees >= -0.5) return true;
        else return false;
    }

    public void end() {
        subsystem.setSpeed(0);
    }
}
