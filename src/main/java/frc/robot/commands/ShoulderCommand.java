package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends CommandBase {
    private static int shoulderSteps;
    private static double encoder;
    
    public ShoulderCommand() {
        encoder = ShoulderSubsystem.getPositionOfEncoder();
    }

    public static void setAngle(int degrees) {
        shoulderSteps = ShoulderSubsystem.getStepsfromDegrees(degrees);
        ShoulderSubsystem.setSpeed(0.25);

        if (encoder >= shoulderSteps) {
            ShoulderSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
       // TODO make this look like a cmd 
    }
 
}
