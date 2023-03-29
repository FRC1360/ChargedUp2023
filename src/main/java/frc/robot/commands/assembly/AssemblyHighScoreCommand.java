package frc.robot.commands.assembly;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

public class AssemblyHighScoreCommand extends SequentialCommandGroup {
    
    public AssemblyHighScoreCommand(ShoulderSubsystem shoulder, ShoulderWristMessenger shoulderWristMessenger, 
                                                WristSubsystem wrist, ArmSubsystem arm, ArmShoulderMessenger armMessenger,
                                                BooleanSupplier scoreCube) { 
        addCommands(
            new InstantCommand( () -> shoulder.setInIntakePosition(false)),

            /*new ShoulderGoToPositionCommand(shoulder, 43)
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm)),

            new WristGoToPositionCommand(wrist, -45.0)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new ArmHoldCommand(arm)), 
    
            new ArmGoToPositionCommand(arm, shoulderWristMessenger, 19.0)
                .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                .raceWith(new WristHoldCommand(wrist, () -> 0.0))*/
            
            new ConditionalCommand(
                // Score Cube
                (new ShoulderGoToPositionCommand(shoulder, 37)
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)))
                .andThen(new WristGoToPositionCommand(wrist, -40.0)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)))
                .andThen(new ArmGoToPositionCommand(arm, shoulderWristMessenger, 19.0)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))), 
                
                //Score Cone
                (new ShoulderGoToPositionCommand(shoulder, 43)
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)))
                .andThen(new WristGoToPositionCommand(wrist, -45.0)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    .raceWith(new ArmHoldCommand(arm)))
                .andThen(new ArmGoToPositionCommand(arm, shoulderWristMessenger, 19.0)
                    .raceWith(new ShoulderHoldCommand(shoulder, armMessenger, () -> 0.0))
                    .raceWith(new WristHoldCommand(wrist, () -> 0.0))),
                
                scoreCube)

            
        ); 
            
    
    }
}