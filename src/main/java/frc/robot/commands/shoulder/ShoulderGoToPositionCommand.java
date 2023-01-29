package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderGoToPositionCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;

    public ShoulderGoToPositionCommand(ShoulderSubsystem shoulder) {
        this.shoulder = shoulder;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.pidController.reset();
    }

    @Override
    public void execute() {
        double target = this.shoulder.getTargetAngle();
        double input = this.shoulder.getShoulderAngle();
        double speed = this.shoulder.pidController.calculate(target, input);

        this.shoulder.setShoulderNormalizedVoltage(speed);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.shoulder.getShoulderAngle() - this.shoulder.getTargetAngle()) < 3;
    }
}
