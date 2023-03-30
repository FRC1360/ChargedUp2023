package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmHomeCommand extends CommandBase {

    private ArmSubsystem arm;
    private boolean homed;

    public ArmHomeCommand(ArmSubsystem arm) {
            this.arm = arm;
            this.homed = false;
            addRequirements(arm);
    }

    @Override
    public void initialize() { 
        this.homed = false; 
    }

    @Override
    public void execute() {
        if (!this.arm.limitSwitch.get()) this.arm.setArmSpeed(-0.2);
        if (this.arm.limitSwitch.get()) {
            this.arm.setArmSpeed(0.0);
            this.arm.resetEncoder();
            this.homed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.homed;
    }
}
