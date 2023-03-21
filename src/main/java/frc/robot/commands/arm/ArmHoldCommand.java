package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoldCommand extends CommandBase {

    private ArmSubsystem arm; 

    public ArmHoldCommand(ArmSubsystem arm) {
        this.arm = arm;

        // If doing Feedforward offset based off the pivot angle, add the pivot to the command params, but DON'T add to requirements,
        // as we don't drive the pivot at all
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.arm.holdPIDController.reset();
    }
    
    @Override
    public void execute() {
        double target = this.arm.getTargetDistance();
        double input = this.arm.getArmDistance();
        double speed = this.arm.holdPIDController.calculate(target, input);

        //this.arm.setArmNormalizedVoltage(speed);  // Probably going to need an offset based off the angle of the arm
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
