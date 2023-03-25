package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class AssemblyGoToConeIntakeCommand extends SequentialCommandGroup {
    
    public AssemblyGoToConeIntakeCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm) { 
        addCommands(new ShoulderGoToPositionCommand(shoulder, -45.0)
            .raceWith(new WristHoldCommand(wrist))
            .raceWith(new ArmHoldCommand(arm)), 

        //             new WristGoToPositionCommand(wrist, 104.0)
        //     .raceWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
        //     .raceWith(new ArmHoldCommand(arm)), 

                    new ArmGoToPositionCommand(arm, shoulderWristMessenger, 10.0)
            .raceWith(new ShoulderHoldCommand(shoulder, () -> 0.0))
            .raceWith(new WristHoldCommand(wrist))
            );
    
    }
}
