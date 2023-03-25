package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SysCheck extends SequentialCommandGroup {
    
    // subs to test
    private ShoulderSubsystem shoulder;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    // trigger to move on to next test
    private Trigger proceed;

    public SysCheck(ShoulderSubsystem shoulder, ArmSubsystem arm, WristSubsystem wrist, Trigger proceed) { 
        this.shoulder = shoulder;
        this.arm = arm;
        this.wrist = wrist;

        this.proceed = proceed;

        addCommands(
            
        );
    }
}
