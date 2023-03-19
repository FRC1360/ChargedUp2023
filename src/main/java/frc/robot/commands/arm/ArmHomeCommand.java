package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmHomeCommand extends CommandBase {

    private ArmSubsystem arm;
    private boolean homed;


    private DigitalInput limitSwitch;

    public ArmHomeCommand(ArmSubsystem arm) {
            this.arm = arm;
            this.homed = false;
            addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.limitSwitch = arm.limitSwitch;
    }

    @Override
    public void execute() {
        if (this.limitSwitch.get()) arm.setArmSpeed(0.25);
        if (!this.limitSwitch.get()) {
            arm.setArmSpeed(0.0);
            arm.resetEncoder();
            homed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return homed;
    }
}
