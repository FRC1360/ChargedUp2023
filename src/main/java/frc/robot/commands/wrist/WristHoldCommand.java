package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristHoldCommand  extends CommandBase{

    private WristSubsystem wrist;

    public WristHoldCommand(WristSubsystem wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        double target = this.wrist.getTargetAngle();
        double input = this.wrist.getWristAngle();
        double speed = this.wrist.pidController.calculate(target, input);

        if(Math.abs(speed) > 0.25) {
            speed = Math.copySign(0.25, speed);
        }
    
        this.wrist.setWristNormalizedVoltage(speed);
         
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
