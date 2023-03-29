package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHoldCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.autoAssembly.AutoAssemblyConeHighScoreCommand;
import frc.robot.commands.shoulder.ShoulderHoldCommand;
import frc.robot.commands.wrist.WristHoldCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmShoulderMessenger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.subsystems.WristSubsystem;

public class ConeScoreHighAuto extends SequentialCommandGroup {
    
    public ConeScoreHighAuto(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                WristSubsystem wrist, ArmSubsystem arm, IntakeSubsystem intake, 
                                    ArmShoulderMessenger armMessenger) 
    {
        addCommands(
            new AutoAssemblyConeHighScoreCommand(shoulder, shoulderWristMessenger, wrist, 
                                                    arm, intake, armMessenger), 
            new AssemblyHomePositionCommand(shoulder, shoulderWristMessenger, wrist, arm, armMessenger), 
            new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm))
            );
    }
}
