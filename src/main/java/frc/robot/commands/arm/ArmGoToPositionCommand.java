package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoToPositionCommand extends CommandBase {

    private ArmSubsystem arm;
    private int position;

    public ArmGoToPositionCommand(ArmSubsystem arm, int position) {
            this.arm = arm;
            
            this.position = position;

            addRequirements(arm);
    }

    public ArmGoToPositionCommand(ArmSubsystem arm, ArmSubsystem.ARM_POSITION position) {
        this(arm, position.getValue());
    }

    @Override
    public void initialize() {
        this.arm.setEncoderTargetPosition(this.position);
        this.arm.pidController.reset();
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
        return (this.arm.getEncoderPosition() - this.arm.getEncoderTargetPosition()) < 5;
    }


    
}
