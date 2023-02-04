package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.Constants;

public class RotateShoulderCommand extends CommandBase {
    private static double degrees = 0.0;
    private static ShoulderSubsystem subsystem;
    private static double shoulderSteps = 0;

    public RotateShoulderCommand(ShoulderSubsystem ssystem, double degrees) {
        subsystem = ssystem;
        this.degrees = degrees; 
        addRequirements(ssystem);
    }

    @Override
    public void initialize() { 
        //subsystem.setZero();
        subsystem.setEncoderTargetPosition(degrees);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("command running", true);
        setAngle(degrees);
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

    public void setAngle(double degrees) {
        //degrees = angle.getValue();
        shoulderSteps = subsystem.getStepsfromAngle(degrees);
        
        double pidoutput = subsystem.pid.calculate(shoulderSteps, subsystem.getPositionOfEncoder());
        if (pidoutput > 0.25) pidoutput = 0.25;
        else if (pidoutput < -0.25) pidoutput = -0.25;

        SmartDashboard.putNumber("speed", pidoutput);
        SmartDashboard.putNumber("input", subsystem.getPositionOfEncoder()); 
        SmartDashboard.putNumber("target", Constants.ROTATIONS_PER_ANGLE_PIVOT * degrees);

        subsystem.setSpeed(pidoutput);
    }
}
