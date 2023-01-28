package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoldCommand extends CommandBase {

    private ArmSubsystem arm; 

    public ArmHoldCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        double target = (double)(this.arm.getEncoderTargetPosition());
        double input = (double)(this.arm.getEncoderPosition());
        double speed = this.arm.pidController.calculate(target, input);

        this.arm.setArmSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
