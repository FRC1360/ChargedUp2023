package frc.robot.commands;
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
        subsystem.setAngle(degrees);
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAngle()-degrees <= 0.1 && subsystem.getAngle()-degrees >= -0.1) return true;
        else return false;
    }

    public void end() {
        subsystem.setSpeed(0);
    }
}
