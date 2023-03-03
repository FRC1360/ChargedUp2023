package frc.robot.commands.assembly;

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

public class AssemblyGoToConeIntakeCommand extends SequentialCommandGroup {
    
    public AssemblyGoToConeIntakeCommand(ShoulderSubsystem shoulder, WristSubsystem wrist, ArmSubsystem arm) { 
        addCommands(new ShoulderGoToPositionCommand(shoulder, 51.0)
            .raceWith(new WristHoldCommand(wrist))
            .raceWith(new ArmHoldCommand(arm)), 

                    new WristGoToPositionCommand(wrist, 125.0)
            .raceWith(new ShoulderHoldCommand(shoulder))
            .raceWith(new ArmHoldCommand(arm)), 

                    new ArmGoToPositionCommand(arm, shoulder, -18.3)
            .raceWith(new ShoulderHoldCommand(shoulder))
            .raceWith(new WristHoldCommand(wrist))
            ); 
    
    }
}
