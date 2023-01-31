package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderCommand extends CommandBase {
    private int shoulderSteps;
    private double encoder;
    
    public ShoulderCommand() {
        encoder = ShoulderSubsystem.getShoulderEncoder().getPosition();
    }

    public void setAngle(int degrees) {
        shoulderSteps = getStepsfromDegrees(degrees);
        ShoulderSubsystem.setSpeed(0.25);

        if (encoder >= shoulderSteps) {
            ShoulderSubsystem.setSpeed(0);
        }
    }

    public double getDegrees() {
        return encoder / Constants.TICKS_PER_ANGLE_PIVOT;
    }

    public int getStepsfromDegrees(int degrees) {
        return Constants.TICKS_PER_ANGLE_PIVOT * degrees;
    }

    public void setTargetLow() {
        setAngle(5); // TODO Enter low target angle
    }
    
    public void setTargetMiddle() {
        setAngle(20); // TODO Enter Middle target angle
    }

    public void setTargetHigh() {
        setAngle(45); // TODO Enter High target angle
    }

    @Override
    public void end(boolean interrupted) {
       // TODO make this look like a cmd 
    }
 
}
