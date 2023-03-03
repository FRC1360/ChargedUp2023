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

public class AssemblyHomePositionCommand extends SequentialCommandGroup {
    
    public AssemblyHomePositionCommand(ShoulderSubsystem shoulder, WristSubsystem wrist, ArmSubsystem arm) { 
        addCommands(new WristGoToPositionCommand(wrist, Constants.WRIST_HOME_ANGLE)
                .raceWith(new ShoulderHoldCommand(shoulder))
                .raceWith(new ArmHoldCommand(arm)), 
            new ArmGoToPositionCommand(arm, 0)
                .raceWith(new ShoulderHoldCommand(shoulder))
                .raceWith(new WristHoldCommand(wrist)), 
            new ShoulderGoToPositionCommand(shoulder, 0)
                .raceWith(new WristHoldCommand(wrist))
                .raceWith(new ArmHoldCommand(arm))
            );
    }
}
