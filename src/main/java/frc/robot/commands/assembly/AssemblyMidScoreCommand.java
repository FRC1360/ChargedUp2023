package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmGoToPositionCommand;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.shoulder.ShoulderGoToPositionCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristGoToPositionCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;

public class AssemblyMidScoreCommand extends SequentialCommandGroup {
    
    public AssemblyMidScoreCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, ArmShoulderMessenger armMessenger) { 
        addCommands(
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),
            
            new ShoulderGoToPositionCommand(shoulder, 30.0)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm)),
    
            new WristGoToPositionCommand(wrist, -45.0)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
        ); 

    }
}
