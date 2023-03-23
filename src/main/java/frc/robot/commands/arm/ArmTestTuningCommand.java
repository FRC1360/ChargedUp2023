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
        
        this.arm.armFeedforward 
            = this.arm.armFeedForwardTuner.getAndUpdate(this.arm.armFeedforward);

        double pidOutput = this.arm.holdPIDController.calculate(10.0, this.arm.getArmDistance());
        
        SmartDashboard.putNumber("Arm_kG", this.arm.armFeedforward.kg); 
        double ff = this.arm.armFeedforward.calculate(-45, 1.0); 
        
        SmartDashboard.putNumber("Arm_Feedforward_Output", ff); 
    }

    @Override
    public boolean isFinished() { 
        return (10 - this.arm.getArmDistance()) < 0.5;  
    }
}
