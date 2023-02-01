package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderGoToPositionCommand extends CommandBase {
    
    private ShoulderSubsystem shoulder;
    private double angle;

    public ShoulderGoToPositionCommand(ShoulderSubsystem shoulder, double angle) {
        this.shoulder = shoulder;
        this.angle = angle;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.pidController.reset();
        this.shoulder.setTargetAngle(angle);
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
        return Math.abs(this.shoulder.getShoulderAngle() - this.shoulder.getTargetAngle()) < 3;
    }
}
