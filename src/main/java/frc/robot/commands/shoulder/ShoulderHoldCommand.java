package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderHoldCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;

    public ShoulderHoldCommand(ShoulderSubsystem shoulder) {
        this.shoulder = shoulder;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.holdPIDController.reset();
    }

    @Override
    public void execute() {

        double target = this.shoulder.getTargetAngle();
        double input = this.shoulder.getShoulderAngle();
        double speed = -this.shoulder.holdPIDController.calculate(target, input);

        // Remember to increase this value and also add kI please
        double kF = -0.18 * Math.sin(Math.toRadians(this.shoulder.getShoulderAngle()));

        this.shoulder.setShoulderNormalizedVoltage(speed + kF);

        SmartDashboard.putNumber("Shoulder_Hold_Output", speed + kF);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
