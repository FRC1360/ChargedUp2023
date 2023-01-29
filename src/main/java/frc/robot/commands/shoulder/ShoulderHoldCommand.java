package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderHoldCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;

    public ShoulderHoldCommand(ShoulderSubsystem shoulder) {
        this.shoulder = shoulder;
        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        double target = this.shoulder.getTargetAngle();
        double input = this.shoulder.getShoulderAngle();
        double speed = this.shoulder.pidController.calculate(target, input);

        if(Math.abs(speed) > 0.5) {
            speed = Math.copySign(0.5, speed);
        }

        this.shoulder.setShoulderNormalizedVoltage(speed);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
