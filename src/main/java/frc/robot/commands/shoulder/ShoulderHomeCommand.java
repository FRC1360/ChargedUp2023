package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;


public class ShoulderHomeCommand extends CommandBase {

    private ShoulderSubsystem shoulder;
    private boolean homed;


    private DigitalInput limitSwitch;

    public ShoulderHomeCommand(ShoulderSubsystem shoulder) {
            this.shoulder = shoulder;
            this.homed = false;
            addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.limitSwitch = new DigitalInput(Constants.LIMIT_SWITCHES.SHOULDER);
    }

    @Override
    public void execute() {
        shoulder.setShoulderSpeed(-0.25);
        if (this.limitSwitch.get()) {
            if (!this.limitSwitch.get()) {
                shoulder.setShoulderSpeed(0);
                return;
            }
        }
        
        shoulder.resetMotorRotations();
        shoulder.setShoulderSpeed(0.1);

        if (!this.limitSwitch.get()) {
            if (this.limitSwitch.get()) {
                shoulder.setShoulderSpeed(0);
                return;
            }
        }

        shoulder.resetMotorRotations();
        homed = true;
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return homed;
        //return false; 
    }


    
}
