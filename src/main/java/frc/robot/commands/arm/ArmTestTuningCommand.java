package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestTuningCommand extends CommandBase {

    private ArmSubsystem arm; 

    public ArmTestTuningCommand(ArmSubsystem arm) { 
        this.arm = arm; 
    }

    @Override
    public void initialize() { 
        this.arm.setTargetDistance(10.0);
    }

    @Override
    public void execute() { 
        this.arm.holdPIDTuner.getAndUpdate(this.arm.holdPIDController);
        SmartDashboard.putNumber("Arm_kP", this.arm.holdPIDController.kP); 

        double val = this.arm.holdPIDController.calculate(10.0, this.arm.getArmDistance()); 
    }

    @Override
    public boolean isFinished() { 
        return (10 - this.arm.getArmDistance()) < 0.5;  
    }
}
