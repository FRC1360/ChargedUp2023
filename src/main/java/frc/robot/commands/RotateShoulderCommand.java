package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.Constants;

public class RotateShoulderCommand extends CommandBase {
    private static double degrees = 0.0;
    private static ShoulderSubsystem shoulder;
    private static double shoulderSteps = 0;

    public RotateShoulderCommand(ShoulderSubsystem ssystem, double degrees) {
        shoulder = ssystem;
        this.degrees = degrees; 
        addRequirements(ssystem);
    }

    @Override
    public void initialize() { 
        //subsystem.setZero();
        shoulder.setEncoderTargetPosition(degrees);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("command running", true);
        setAngle(degrees);
        SmartDashboard.putNumber("current angle", shoulder.getAngle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shoulder.getAngle()-degrees) <= 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("command running", false);
        shoulder.setSpeed(0);
    }

    public void setAngle(double degrees) {
        //degrees = angle.getValue();
        shoulderSteps = shoulder.getStepsfromAngle(degrees);
        
        double pidoutput = shoulder.pid.calculate(shoulderSteps, shoulder.getPositionOfEncoder());
        if (pidoutput > 0.25) pidoutput = 0.25;
        else if (pidoutput < -0.25) pidoutput = -0.25;

        SmartDashboard.putNumber("speed", pidoutput);
        SmartDashboard.putNumber("input", shoulder.getPositionOfEncoder()); 
        SmartDashboard.putNumber("target", Constants.ROTATIONS_PER_ANGLE_PIVOT * degrees);

        shoulder.setSpeed(pidoutput);
    }
}
