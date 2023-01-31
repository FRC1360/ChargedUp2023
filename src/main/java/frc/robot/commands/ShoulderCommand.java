package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends CommandBase {
    private static int shoulderSteps;
    private static double encoder;
    private static ShoulderSubsystem subsystem;

    public ShoulderCommand(ShoulderSubsystem ssystem) {
        subsystem = ssystem;
        addRequirements(ssystem);
    }

    @Override
    public void initialize() { 
        encoder = subsystem.getPositionOfEncoder();
    }

    public static void setAngle(int degrees) {
        shoulderSteps = subsystem.getStepsfromDegrees(degrees);
        subsystem.setSpeed(0.25);

        if (encoder >= shoulderSteps) {
            subsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
       // TODO make this look like a cmd 
    }
 
}
